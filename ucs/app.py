from flask import Flask, render_template
from os import getenv

app = Flask(__name__)
app.config["SECRET_KEY"] = getenv("FLASK_SECRET_KEY")

if not app.config["SECRET_KEY"]:
    raise ValueError("No FLASK_SECRET_KEY set - Please set the secret key before continuing")

def registerRoutes() -> None:
	@app.route("/")
	def index():
		return render_template("index.html")

registerRoutes()
