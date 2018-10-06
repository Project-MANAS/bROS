import redis

def setData(key, value):

    assert isinstance(key, str), "Expected key to be of type str"
    redis.Redis().set(key, value)

def getData(key, defaultValue = None):

    assert isinstance(key, str), "Expected key to be of type str"
    value = redis.Redis().get(key)

    if value == None:
        if not defaultValue:
            defaultValue = 0
        setData(key, defaultValue)
        value = defaultValue
    return value
