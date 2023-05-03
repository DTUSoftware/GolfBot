from flask import Flask, Blueprint

bp = Blueprint('ev3', __name__)


@bp.route('/drive_forward')
def drive_forward():
    return 'forward'


@bp.route('/drive_backwards')
def drive_backwards():
    return 'backwards'


app = Flask(__name__)
app.register_blueprint(bp, url_prefix='/api/v1')

if __name__ == '__main__':
    app.run()
