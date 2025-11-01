class Config():
    DEBUG = False
    SQL_ALCHEMY_TRACK_MODIFICATIONS = False

class LocalDevelopmentConfig(Config):
    SQLALCHEMY_DATABASE_URI = "sqlite:///database.sqlite3"
    DEBUG = True
    SECURITY_REGISTRABLE = True
    SECURITY_TRACKABLE = True
    SECURITY_PASSWORD_HASH = 'bcrypt'
    SECURITY_PASSWORD_SALT = 'opticaldesignsetup'
    SECRET_KEY = 'asecretkey'
    SECURITY_TOKEN_AUTHENTICATION_HEADER = 'Authorization'

    WTF_CSRF_ENABLED = False
    