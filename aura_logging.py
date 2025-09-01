# aura_logging.py
import logging
import os
import sys
from logging.handlers import RotatingFileHandler
from pathlib import Path


class ColorFormatter(logging.Formatter):
    LOG_COLORS = {
        "DEBUG": "\033[94m",
        "INFO": "\033[92m",
        "WARNING": "\033[93m",
        "ERROR": "\033[91m",
        "CRITICAL": "\033[1;91m",
        "RESET": "\033[0m",
    }

    def format(self, record):
        log_format = (
            f"%(asctime)s - "
            f"{self.LOG_COLORS.get(record.levelname, '')}%(levelname)s\033[0m - "
            f"%(name)s - %(message)s"
        )
        formatter = logging.Formatter(log_format, "%Y-%m-%d %H:%M:%S")
        return formatter.format(record)


def setup_logger(
    name: str | None = None,
    level: str = "INFO",
    log_file: Path | None = None,
):
    log_level_str = os.getenv("AURA_LOG_LEVEL", level).upper()
    log_level = getattr(logging, log_level_str, logging.INFO)
    logger = logging.getLogger(name)
    logger.setLevel(log_level)
    logger.propagate = False
    if logger.hasHandlers():
        logger.handlers.clear()
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setLevel(log_level)
    console_handler.setFormatter(ColorFormatter())
    logger.addHandler(console_handler)
    if log_file:
        log_file.parent.mkdir(parents=True, exist_ok=True)
        file_handler = RotatingFileHandler(
            log_file, maxBytes=5 * 1024 * 1024, backupCount=5
        )
        file_handler.setLevel(log_level)
        file_handler.setFormatter(
            logging.Formatter(
                "%(asctime)s - %(levelname)s - %(name)s - %(message)s",
                "%Y-%m-%d %H:%M:%S",
            )
        )
        logger.addHandler(file_handler)

    def handle_exception(exc_type, exc_value, exc_traceback):
        if issubclass(exc_type, KeyboardInterrupt):
            sys.__excepthook__(exc_type, exc_value, exc_traceback)
            return
        logger.error(
            "Uncaught exception", exc_info=(exc_type, exc_value, exc_traceback)
        )

    sys.excepthook = handle_exception
    return logger
