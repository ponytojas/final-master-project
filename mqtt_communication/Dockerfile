FROM python:3.8.18-bookworm
WORKDIR /app

COPY ./node .

RUN rm .env

RUN pip3 install --no-cache-dir -r requirements.txt

CMD ["python", "main.py"]
