import logging
import os
import sys

# If logging should be disabled
DISABLE_LOGGING = "true" in os.environ.get('DISABLE_LOGGING', "True").lower()
# If debugging should be enabled
DEBUG = "true" in os.environ.get('DEBUG', "True").lower()
LOGGER = None


def get_logger() -> logging.Logger:
    global LOGGER
    if not LOGGER:
        LOGGER = configure_logger()
    return LOGGER


def configure_logger() -> logging.Logger:
    logger = logging.getLogger("GolfBot")
    logger.addHandler(logging.StreamHandler(sys.stdout))
    if DEBUG:
        logger.setLevel(logging.DEBUG)
    if DISABLE_LOGGING:
        logger.setLevel(logging.CRITICAL)
    return logger
