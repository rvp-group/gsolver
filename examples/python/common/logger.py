"""
Lightweight logging utility for gsolver Python experiments.

Provides structured logging with severity levels, timestamps, and consistent formatting.
Mirrors the C++ logger.h for consistent output across languages.

Usage:
    from common.logger import Logger, LOG_INFO, LOG_WARN, LOG_ERROR

    LOG_SET_MODULE("PGO")
    LOG_INFO("Processing {} poses", num_poses)
    LOG_STEP(1, 4, "Loading data from: {}", input_path)
"""

import sys
from datetime import datetime
from enum import IntEnum
from typing import Any


class LogLevel(IntEnum):
    DEBUG = 0
    INFO = 1
    WARN = 2
    ERROR = 3
    FATAL = 4


# ANSI color codes
_COLORS = {
    LogLevel.DEBUG: "\033[36m",   # Cyan
    LogLevel.INFO: "\033[32m",    # Green
    LogLevel.WARN: "\033[33m",    # Yellow
    LogLevel.ERROR: "\033[31m",   # Red
    LogLevel.FATAL: "\033[35m",   # Magenta
}
_RESET = "\033[0m"

_LEVEL_NAMES = {
    LogLevel.DEBUG: "DEBUG",
    LogLevel.INFO: "INFO ",
    LogLevel.WARN: "WARN ",
    LogLevel.ERROR: "ERROR",
    LogLevel.FATAL: "FATAL",
}


class Logger:
    """Singleton logger with colored output and progress tracking."""

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._init()
        return cls._instance

    def _init(self):
        self.min_level = LogLevel.INFO
        self.color_enabled = True
        self.module_name = ""

    def set_level(self, level: LogLevel):
        self.min_level = level

    def set_color_enabled(self, enabled: bool):
        self.color_enabled = enabled

    def set_module_name(self, name: str):
        self.module_name = name

    def _format_message(self, fmt: str, *args: Any) -> str:
        """Format message with {} placeholders."""
        result = fmt
        for arg in args:
            result = result.replace("{}", str(arg), 1)
        return result

    def _get_timestamp(self) -> str:
        """Get formatted timestamp."""
        now = datetime.now()
        return now.strftime("%Y-%m-%d %H:%M:%S") + f".{now.microsecond // 1000:03d}"

    def log(self, level: LogLevel, fmt: str, *args: Any):
        """Log a message at the given level."""
        if level < self.min_level:
            return

        message = self._format_message(fmt, *args)
        timestamp = self._get_timestamp()

        parts = []
        if self.color_enabled:
            parts.append(_COLORS.get(level, ""))

        parts.append(f"[{timestamp}] ")
        parts.append(f"[{_LEVEL_NAMES[level]}] ")

        if self.module_name:
            parts.append(f"[{self.module_name}] ")

        parts.append(message)

        if self.color_enabled:
            parts.append(_RESET)

        output = "".join(parts)
        if level >= LogLevel.ERROR:
            print(output, file=sys.stderr)
        else:
            print(output)

    def section(self, title: str):
        """Print a section header."""
        separator = "=" * 60
        print(f"\n{separator}")
        print(f"  {title}")
        print(f"{separator}\n")

    def step(self, num: int, total: int, fmt: str, *args: Any):
        """Log a step in a multi-step process."""
        desc = self._format_message(fmt, *args)
        self.log(LogLevel.INFO, f"[{num}/{total}] {desc}")

    def progress(self, task: str, current: int, total: int):
        """Show a progress bar."""
        if total <= 0:
            return

        percent = current * 100 // total
        bar_width = 30
        filled = bar_width * current // total

        timestamp = self._get_timestamp()

        parts = []
        if self.color_enabled:
            parts.append(_COLORS[LogLevel.INFO])

        parts.append(f"\r[{timestamp}] ")
        parts.append(f"[{_LEVEL_NAMES[LogLevel.INFO]}] ")

        if self.module_name:
            parts.append(f"[{self.module_name}] ")

        parts.append(f"{task} [")
        parts.append("█" * filled)
        parts.append("░" * (bar_width - filled))
        parts.append(f"] {percent}% ({current}/{total})")

        if self.color_enabled:
            parts.append(_RESET)

        print("".join(parts), end="", flush=True)
        if current == total:
            print()


# Singleton instance
_logger = Logger()


# Convenience functions (mirroring C++ macros)
def LOG_DEBUG(fmt: str, *args: Any):
    _logger.log(LogLevel.DEBUG, fmt, *args)


def LOG_INFO(fmt: str, *args: Any):
    _logger.log(LogLevel.INFO, fmt, *args)


def LOG_WARN(fmt: str, *args: Any):
    _logger.log(LogLevel.WARN, fmt, *args)


def LOG_ERROR(fmt: str, *args: Any):
    _logger.log(LogLevel.ERROR, fmt, *args)


def LOG_FATAL(fmt: str, *args: Any):
    _logger.log(LogLevel.FATAL, fmt, *args)


def LOG_PROGRESS(current: int, total: int, task: str):
    _logger.progress(task, current, total)


def LOG_SECTION(title: str):
    _logger.section(title)


def LOG_STEP(num: int, total: int, fmt: str, *args: Any):
    _logger.step(num, total, fmt, *args)


def LOG_SET_LEVEL(level: LogLevel):
    _logger.set_level(level)


def LOG_SET_MODULE(name: str):
    _logger.set_module_name(name)


def LOG_SET_COLOR(enabled: bool):
    _logger.set_color_enabled(enabled)
