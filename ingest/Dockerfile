FROM python:3.11.15-slim
WORKDIR /app
ENV PYTHONUNBUFFERED=1
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY mqtt_ingest.py .
CMD ["python3", "mqtt_ingest.py"]
