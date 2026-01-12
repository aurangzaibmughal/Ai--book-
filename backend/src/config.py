"""
Configuration management for the robotics textbook backend.
Loads environment variables and provides application settings.
"""

from pydantic_settings import BaseSettings
from typing import List


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # OpenAI Configuration
    openai_api_key: str
    openai_model: str = "gpt-4"
    embedding_model: str = "text-embedding-ada-002"

    # Database Configuration
    database_url: str

    # Qdrant Configuration
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection_name: str = "textbook_content"

    # Application Settings
    environment: str = "development"
    log_level: str = "INFO"
    debug: bool = True

    # CORS Settings
    cors_origins: str = "http://localhost:3000"
    cors_allow_credentials: bool = True
    cors_allow_methods: str = "*"
    cors_allow_headers: str = "*"

    # API Settings
    api_v1_prefix: str = "/v1"
    max_conversation_length: int = 50
    rate_limit_per_minute: int = 60

    # Security
    secret_key: str
    algorithm: str = "HS256"
    access_token_expire_minutes: int = 30

    @property
    def cors_origins_list(self) -> List[str]:
        """Parse CORS origins from comma-separated string."""
        return [origin.strip() for origin in self.cors_origins.split(",")]

    class Config:
        env_file = ".env"
        case_sensitive = False


# Global settings instance
settings = Settings()
