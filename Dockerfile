FROM python:3.14-slim

WORKDIR /src

COPY ./visualisation/requirements.txt ./

RUN pip install --no-cache-dir -r requirements.txt

COPY ./visualisation ./visualisation

WORKDIR /src/visualisation
CMD ["python", "launcher.py"]