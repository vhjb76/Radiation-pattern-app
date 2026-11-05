"""Entry point for the TITAN-IMX demo application."""
import logging
import tkinter as tk
from pathlib import Path

from .gui import TitanDemoApp

LOG_PATH = Path.home() / "titan_imx_demo.log"


def main() -> None:
    logging.basicConfig(
        filename=LOG_PATH,
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )
    root = tk.Tk()
    TitanDemoApp(root)
    try:
        root.mainloop()
    except Exception:
        logging.exception("Unhandled exception in main loop")
        raise


if __name__ == "__main__":
    main()
