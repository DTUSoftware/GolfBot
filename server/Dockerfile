FROM python:3.5.3

##RUN echo 'http://dl-cdn.alpinelinux.org/alpine/v3.15/main' >> /etc/apk/repositories
#RUN apk update
## https://pillow.readthedocs.io/en/latest/installation.html#external-libraries
#RUN apk add tiff-dev jpeg-dev openjpeg-dev zlib-dev freetype-dev lcms2-dev libwebp-dev tcl-dev tk-dev harfbuzz-dev \
# fribidi-dev libimagequant-dev libxcb-dev libpng-dev python3-dev gcc libc-dev libffi-dev \
# --no-cache --allow-untrusted --repository http://dl-cdn.alpinelinux.org/alpine/v3.15/main \
# --repository https://alpine.global.ssl.fastly.net/alpine/edge/community \
# --repository https://alpine.global.ssl.fastly.net/alpine/edge/main
## --repository https://dl-4.alpinelinux.org/alpine/edge/testing

WORKDIR /usr/src/app

COPY requirements.txt .
RUN python -m pip install --no-cache-dir -r requirements.txt

COPY . .

EXPOSE 80

CMD ["gunicorn", "--bind", "0.0.0.0:80", "app:app"]
