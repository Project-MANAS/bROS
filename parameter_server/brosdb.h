#pragma once
#ifndef BROS_PARAMETER_SERVER_BROSDB_
#define BROS_PARAMETER_SERVER_BROSDB_

#include <string>
#include <cstring>
#include "hiredis/hiredis.h"

namespace brosdb {

template<typename T>
void set(std::string key, T value) {
  redisContext *context = redisConnect("127.0.0.1", 6379);
  redisReply *reply = (redisReply *) redisCommand(context, "SET %b %b", key.c_str(), key.length(), &value, sizeof(T));
  freeReplyObject(reply);
}

template<typename T>
void get(std::string key, T *value, T defaultValue) {
  redisContext *context = redisConnect("127.0.0.1", 6379);
  redisReply *reply = (redisReply *) redisCommand(context, "GET %b", key.c_str(), key.length());
  if (reply->str == nullptr) {
    *value = defaultValue;
    set(key, defaultValue);
  } else {
    std::memcpy(value, reply->str, sizeof(T));
  }
  freeReplyObject(reply);
}

template<typename T>
void get(std::string key, T *value) {
  get(key, value, (T) 0);
}

}

#endif
