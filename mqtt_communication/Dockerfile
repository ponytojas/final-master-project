FROM python:3.12.2-alpine3.19
WORKDIR /app

COPY ./src .

RUN pip install --no-cache-dir -r requirements.txt

CMD ["python", "main.py"]
