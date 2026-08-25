FROM python:3.11.15-slim
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY nav_backend.py .
EXPOSE 9000
CMD ["gunicorn", "-w", "2", "-b", "0.0.0.0:9000", "nav_backend:app"]
