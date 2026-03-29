# Use an official Python runtime as a parent image
FROM python:3.9-slim

# Set the working directory in the container
WORKDIR /app

# Install system dependencies required for psycopg2 and other packages
RUN apt-get update && apt-get install -y \
    gcc \
    libpq-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy the requirements file into the container at /app
COPY requirements.txt ./

# Install any needed packages specified in requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

# Use a build argument to manually bust the cache from this layer onwards
# Usage: docker compose build --build-arg CACHEBUST=$(date +%s)
ARG CACHEBUST=1

# Copy the current directory contents into the container at /app
COPY . .

# Set python path to allow imports from root and submodules
ENV PYTHONPATH=/app:/app/submodules

# Default command
CMD ["python3", "fleet_management/FmMain.py"]
