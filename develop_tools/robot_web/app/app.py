from flask import Flask, render_template
app = Flask(__name__)

def createApp():
    return app

@app.route("/")
def hello():
    return render_template('index.html')