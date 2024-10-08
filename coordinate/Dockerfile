FROM python:3.9

WORKDIR /app

COPY ./requirements.txt /app/requirements.txt

RUN apt-get update -y
RUN apt-get install -y libgl1-mesa-glx
RUN apt-get install -y libglib2.0-0
RUN pip install --no-cache-dir --upgrade -r /app/requirements.txt

COPY ./Python_Server.py /app

CMD [ "python3", "Python_Server.py" ]

