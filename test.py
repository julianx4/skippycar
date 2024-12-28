import redis
import time

redis_client = redis.Redis(host='localhost', port=6379, db=0)

redis_client.set('angle', 0)