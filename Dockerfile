FROM python:3.12-bookworm

WORKDIR /opt/CeasearKit/src/

COPY requirements.txt ./
RUN pip install -r requirements.txt

COPY ..src ./src
CMD ["python", "/main.py"]