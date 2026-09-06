import argparse
import sys
from PySide6 import QtCore, QtGui, QtWidgets
from app.ui.window import MainWindow


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--mock",action="store_true")
    parser.add_argument("--fresh",action="store_true")
    parser.add_argument("--smoke",action="store_true",help="Run Mock, capture all pages, then exit")
    args=parser.parse_args()
    fmt=QtGui.QSurfaceFormat();fmt.setDepthBufferSize(24);fmt.setSamples(4);QtGui.QSurfaceFormat.setDefaultFormat(fmt)
    app=QtWidgets.QApplication(sys.argv);app.setApplicationName("Embedded Station")
    window=MainWindow(restore=not args.fresh and not args.smoke);window.show()
    if args.mock or args.smoke:window.connect_mock()
    if args.smoke:
        from pathlib import Path
        output=(Path(sys.executable).resolve().parent if getattr(sys,"frozen",False) else Path(__file__).resolve().parents[1])/"docs/screenshots";output.mkdir(parents=True,exist_ok=True)
        def capture(i=0):
            if i>=window.nav.count():
                window.engine.event("SMOKE","GUI pages captured")
                window.close();app.quit();return
            window.nav.setCurrentRow(i)
            def save():
                window.grab().save(str(output/f"{i:02d}.png"));capture(i+1)
            QtCore.QTimer.singleShot(600,save)
        QtCore.QTimer.singleShot(4500,capture)
    return app.exec()


if __name__=="__main__":raise SystemExit(main())
