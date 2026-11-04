"""RabbitMQ credentials are local configuration, never repository defaults."""
import json
import os
from pathlib import Path


def rabbitmq_credentials():
    path = Path(os.environ.get('FOD_PLATFORM_CREDENTIALS', str(
        Path(__file__).resolve().parents[1]/'config/platform_credentials.local.json')))
    values = json.loads(path.read_text()) if path.exists() else {}
    return (os.environ.get('RABBITMQ_USER', values.get('username', '')),
            os.environ.get('RABBITMQ_PWD', values.get('password', '')))
