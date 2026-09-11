import argparse
import csv
from dataclasses import dataclass
from math import cos, radians, sin
from pathlib import Path

import yaml
from PyQt5.QtCore import QPointF, Qt
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


@dataclass(frozen=True)
class PolarCell:
    angle_deg: float
    range_m: float
    persistence: float
    z_min_m: float
    z_max_m: float


def load_cells(path: str):
    cells = []
    with Path(path).expanduser().open('r', encoding='utf-8', newline='') as f:
        for row in csv.DictReader(f):
            cells.append(PolarCell(
                angle_deg=float(row['angle_center_deg']),
                range_m=float(row['range_center_m']),
                persistence=float(row['persistence']),
                z_min_m=float(row['z_min_m']),
                z_max_m=float(row['z_max_m']),
            ))
    if not cells:
        raise RuntimeError('polar CSV contains no cells')
    return cells


def load_suggestion(path: str):
    data = yaml.safe_load(Path(path).expanduser().read_text(encoding='utf-8')) or {}
    if data.get('status') != 'candidate':
        return None
    params = data.get('runtime_parameter_snippet', {})
    return {
        'center_deg': float(params.get('filters.rear_sector.center_deg', 180.0)),
        'width_deg': float(params.get('filters.rear_sector.width_deg', 10.0)),
        'min_range_m': float(params.get('filters.rear_sector.min_range_m', 0.2)),
        'max_range_m': float(params.get('filters.rear_sector.max_range_m', 2.0)),
        'z_min_m': float(params.get('filters.rear_sector.z_min_m', -0.5)),
        'z_max_m': float(params.get('filters.rear_sector.z_max_m', 1.5)),
    }


def initial_from_cells(cells):
    min_r = min(c.range_m for c in cells)
    max_r = max(c.range_m for c in cells)
    min_z = min(c.z_min_m for c in cells)
    max_z = max(c.z_max_m for c in cells)

    # Sparse diagnostic CSVs can contain one cell only. Keep the initial
    # geometry editable and valid instead of collapsing min/max to the same
    # value and presenting an empty/invalid tuner.
    r_pad = max(0.10, 0.10 * max(max_r, 1.0))
    z_pad = 0.10
    return {
        'center_deg': 180.0,
        'width_deg': 20.0,
        'min_range_m': max(0.0, min(0.2, min_r - r_pad)),
        'max_range_m': max(max_r + r_pad, 0.5),
        'z_min_m': min_z - z_pad,
        'z_max_m': max_z + z_pad,
    }


def angular_delta(value, center):
    return ((value - center + 180.0) % 360.0) - 180.0


class PolarView(QWidget):
    def __init__(self, cells, parent=None):
        super().__init__(parent)
        self.cells = cells
        self.params = {
            'center_deg': 180.0,
            'width_deg': 10.0,
            'min_range_m': 0.2,
            'max_range_m': 2.0,
            'z_min_m': -0.5,
            'z_max_m': 1.5,
        }
        self.setMinimumSize(520, 520)

    def set_params(self, params):
        self.params = dict(params)
        self.update()

    def _xy(self, angle_deg, range_m, cx, cy, scale):
        a = radians(angle_deg)
        return QPointF(cx + range_m * cos(a) * scale,
                       cy - range_m * sin(a) * scale)

    def selected_stats(self):
        p = self.params
        selected = [
            c for c in self.cells
            if abs(angular_delta(c.angle_deg, p['center_deg'])) <= 0.5 * p['width_deg']
            and p['min_range_m'] <= c.range_m <= p['max_range_m']
            and c.z_max_m >= p['z_min_m']
            and c.z_min_m <= p['z_max_m']
        ]
        if not selected:
            return 0, 0.0
        return len(selected), sum(c.persistence for c in selected) / len(selected)

    def paintEvent(self, _event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor(30, 30, 30))

        margin = 35
        w, h = self.width(), self.height()
        cx, cy = w * 0.5, h * 0.5
        max_r = max(max(c.range_m for c in self.cells), self.params['max_range_m'], 0.1)
        scale = max(1.0, (min(w, h) * 0.5 - margin) / max_r)

        painter.setPen(QPen(QColor(90, 90, 90), 1))
        for frac in (0.25, 0.5, 0.75, 1.0):
            r = max_r * frac * scale
            painter.drawEllipse(QPointF(cx, cy), r, r)
            painter.drawText(int(cx + 5), int(cy - r + 15), f'{max_r * frac:.1f} m')

        painter.setPen(QPen(QColor(120, 120, 120), 1))
        painter.drawLine(int(cx), margin, int(cx), h - margin)
        painter.drawLine(margin, int(cy), w - margin, int(cy))
        painter.drawText(int(cx + max_r * scale - 35), int(cy - 8), 'front 0°')
        painter.drawText(int(cx - max_r * scale + 5), int(cy - 8), 'rear 180°')

        z_min = self.params['z_min_m']
        z_max = self.params['z_max_m']
        for cell in self.cells:
            if cell.z_max_m < z_min or cell.z_min_m > z_max:
                continue
            pt = self._xy(cell.angle_deg, cell.range_m, cx, cy, scale)
            alpha = int(50 + 205 * max(0.0, min(1.0, cell.persistence)))
            color = QColor(255, 180, 40, alpha)
            painter.setPen(Qt.NoPen)
            painter.setBrush(color)
            painter.drawEllipse(pt, 2.5, 2.5)

        p = self.params
        painter.setPen(QPen(QColor(255, 80, 80), 3))
        half = 0.5 * p['width_deg']
        for angle in (p['center_deg'] - half, p['center_deg'] + half):
            a = self._xy(angle, p['min_range_m'], cx, cy, scale)
            b = self._xy(angle, p['max_range_m'], cx, cy, scale)
            painter.drawLine(a, b)

        steps = max(8, int(p['width_deg'] / 2.0))
        for radius in (p['min_range_m'], p['max_range_m']):
            prev = None
            for i in range(steps + 1):
                angle = p['center_deg'] - half + p['width_deg'] * i / steps
                pt = self._xy(angle, radius, cx, cy, scale)
                if prev is not None:
                    painter.drawLine(prev, pt)
                prev = pt

        painter.setBrush(QColor(220, 220, 220))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(QPointF(cx, cy), 6, 6)


class TunerWindow(QMainWindow):
    def __init__(self, cells, initial, output_path):
        super().__init__()
        self.cells = cells
        self.output_path = Path(output_path).expanduser()
        self.setWindowTitle('AGT LiDAR Filter Tuner')

        root = QWidget()
        layout = QHBoxLayout(root)
        self.view = PolarView(cells)
        layout.addWidget(self.view, 1)

        side = QWidget()
        side_layout = QVBoxLayout(side)
        form = QFormLayout()

        self.center = self._spin(0.0, 360.0, initial['center_deg'], 1.0)
        self.width = self._spin(1.0, 180.0, initial['width_deg'], 1.0)
        self.r_min = self._spin(0.0, 20.0, initial['min_range_m'], 0.05)
        self.r_max = self._spin(0.0, 20.0, initial['max_range_m'], 0.05)
        self.z_min = self._spin(-5.0, 5.0, initial['z_min_m'], 0.05)
        self.z_max = self._spin(-5.0, 5.0, initial['z_max_m'], 0.05)

        form.addRow('Center (deg)', self.center)
        form.addRow('Width (deg)', self.width)
        form.addRow('Min range (m)', self.r_min)
        form.addRow('Max range (m)', self.r_max)
        form.addRow('Z min (m)', self.z_min)
        form.addRow('Z max (m)', self.z_max)
        side_layout.addLayout(form)

        self.stats = QLabel()
        self.stats.setWordWrap(True)
        side_layout.addWidget(self.stats)

        save = QPushButton('Save runtime YAML')
        save.clicked.connect(self.save_yaml)
        side_layout.addWidget(save)

        save_as = QPushButton('Save as...')
        save_as.clicked.connect(self.save_as)
        side_layout.addWidget(save_as)
        side_layout.addStretch(1)

        layout.addWidget(side)
        self.setCentralWidget(root)

        for box in (self.center, self.width, self.r_min, self.r_max, self.z_min, self.z_max):
            box.valueChanged.connect(self.refresh)
        self.refresh()

    @staticmethod
    def _spin(lo, hi, value, step):
        box = QDoubleSpinBox()
        box.setRange(lo, hi)
        box.setDecimals(3)
        box.setSingleStep(step)
        box.setValue(value)
        return box

    def params(self):
        return {
            'center_deg': self.center.value(),
            'width_deg': self.width.value(),
            'min_range_m': self.r_min.value(),
            'max_range_m': self.r_max.value(),
            'z_min_m': self.z_min.value(),
            'z_max_m': self.z_max.value(),
        }

    def refresh(self):
        p = self.params()
        valid = p['min_range_m'] < p['max_range_m'] and p['z_min_m'] < p['z_max_m']
        self.view.set_params(p)
        count, mean_p = self.view.selected_stats()
        self.stats.setText(
            f'Selected polar cells: {count}\n'
            f'Mean persistence: {mean_p:.3f}\n'
            f'Geometry valid: {"yes" if valid else "NO"}')
        self.stats.setStyleSheet('color: #dddddd;' if valid else 'color: #ff7070;')

    def _payload(self):
        p = self.params()
        if p['min_range_m'] >= p['max_range_m'] or p['z_min_m'] >= p['z_max_m']:
            raise RuntimeError('min values must be lower than max values')
        return {
            'agt_pointcloud_filter': {
                'ros__parameters': {
                    # Self-contained preview profile. Merge these rear-sector\n                    # parameters into navigation.yaml after full-bag acceptance.\n                    'filter_chain': ['rear_sector'],
                    'filters.rear_sector.type':
                        'agt_pointcloud_pipeline/SectorFilterPlugin',
                    'filters.rear_sector.center_deg': p['center_deg'],
                    'filters.rear_sector.width_deg': p['width_deg'],
                    'filters.rear_sector.min_range_m': p['min_range_m'],
                    'filters.rear_sector.max_range_m': p['max_range_m'],
                    'filters.rear_sector.z_min_m': p['z_min_m'],
                    'filters.rear_sector.z_max_m': p['z_max_m'],
                }
            }
        }

    def save_yaml(self):
        try:
            self.output_path.parent.mkdir(parents=True, exist_ok=True)
            self.output_path.write_text(
                yaml.safe_dump(self._payload(), sort_keys=False),
                encoding='utf-8')
            QMessageBox.information(self, 'Saved', str(self.output_path))
        except Exception as exc:
            QMessageBox.critical(self, 'Save failed', str(exc))

    def save_as(self):
        chosen, _ = QFileDialog.getSaveFileName(
            self, 'Save runtime YAML', str(self.output_path), 'YAML (*.yaml *.yml)')
        if chosen:
            self.output_path = Path(chosen)
            self.save_yaml()


def _parser():
    p = argparse.ArgumentParser(description='Interactive polar LiDAR filter tuner.')
    p.add_argument('--csv', required=True, help='*_polar.csv from agt-lidar-analyze')
    p.add_argument('--suggestion', help='*_suggested_filter.yaml from analyzer')
    p.add_argument('--output', default='tuned_filter.yaml')
    return p


def main():
    args = _parser().parse_args()
    cells = load_cells(args.csv)
    suggestion = load_suggestion(args.suggestion) if args.suggestion else None
    initial = suggestion if suggestion is not None else initial_from_cells(cells)

    app = QApplication([])
    window = TunerWindow(cells, initial, args.output)
    window.resize(980, 620)
    window.show()
    raise SystemExit(app.exec_())
