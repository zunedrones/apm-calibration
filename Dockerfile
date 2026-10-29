FROM python:3.11-slim
ENV PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
WORKDIR /opt/apm
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt pytest
COPY app ./app
COPY scripts ./scripts
COPY tests ./tests
EXPOSE 8080
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8080"]
