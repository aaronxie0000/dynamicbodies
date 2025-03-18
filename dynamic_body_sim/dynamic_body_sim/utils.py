import logging

# Define ANSI escape sequences for colors
RESET = "\033[0m"
YELLOW = "\033[93m"
RED = "\033[91m"
BLUE = "\033[94m"

class CustomFormatter(logging.Formatter):
    """Custom logging formatter to colorize log messages based on severity."""
    
    def format(self, record):
        # Customize the color based on the log level
        if record.levelno == logging.INFO:
            levelname_color = f"{BLUE}{record.levelname}{RESET}"
            record.msg = f"{BLUE}{record.msg}{RESET}"
        elif record.levelno == logging.WARNING:
            levelname_color = f"{RED}{record.levelname}{RESET}"
            record.msg = f"{RED}{record.msg}{RESET}"
        else:
            levelname_color = record.levelname
        
        # Update the levelname in the record
        record.levelname = levelname_color
        return super().format(record)

def setup_logger(name):
    """Setup logger with the custom formatter."""
    logger = logging.getLogger(name)
    handler = logging.StreamHandler()
    handler.setFormatter(CustomFormatter(
        fmt="%(levelname)s: [%(name)s] %(message)s"
    ))
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)  # Set default level to INFO
    logger.propagate = False
    return logger
