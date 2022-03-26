import io
from flask import Flask, render_template, jsonify, request
import redis
import numpy as np
import struct

app = Flask(__name__)
r = redis.Redis()

variables = []

@app.route("/")
def index():
    return render_template("index.jinja")

@app.route("/variables")
def api_variables():
    return jsonify(variables)

@app.route("/value", methods=['GET', 'POST'])
def api_value():
    if request.method == 'POST':
        value = request.json['value']
        name = request.json['name']
        r.set(name, str(value))
        return ''
    else:
        name = request.values.get('name')
        value = r.get(name) or 0
        return jsonify(value)



@app.route("/save", methods=['POST'])
def save():
    with io.open('variables.txt', 'w', encoding='utf8') as f:
        for name in variables:
            value = r.get(name) or b'0'
            f.write('%s = %s\n' % (
                name,
                value.decode('utf8')
            ))
    return ''

def load_and_set_from_txt():
    with io.open('variables.txt', 'r', encoding='utf8') as f:
        for line in f:
            name, value = line.strip().split('=')
            name = name.strip()
            value = float(value)
            r.set(name, value)
            variables.append(name)
    #variables.sort()


load_and_set_from_txt()
