FROM python:3.8.18-bookworm
WORKDIR /app

COPY ./src .

RUN rm .env

RUN pip3 install --no-cache-dir -r requirements.txt

CMD ["python", "main.py"]
