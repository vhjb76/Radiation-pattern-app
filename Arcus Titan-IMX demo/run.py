"""Standalone entry point so PyInstaller doesn't run main.py as a package-less script."""
from titan_imx_demo.main import main

if __name__ == "__main__":
    main()
