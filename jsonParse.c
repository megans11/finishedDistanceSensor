#include "jsmn.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/*
 * A small example of jsmn parsing when JSON structure is known and number of
 * tokens is predictable.
 */

//static const char *JSON_STRING =
  //  "{\"type\": \"shooter\", \"action\": motorOn}";

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
  if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
      strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
    return 0;
  }
  return -1;
}



int parseAction(char* JSON_STRING, char* myType, char* myAction, char* myBoard, int* myCount)
{
  int i;
  int r;
  jsmn_parser p;
  jsmntok_t t[128]; /* We expect no more than 128 tokens */
  char count_buff[10];
  jsmn_init(&p);
  r = jsmn_parse(&p, JSON_STRING, strlen(JSON_STRING), t, sizeof(t) / sizeof(t[0]));
  if (r < 0) {
    return -1;
  }

  /* Assume the top-level element is an object */
  if (r < 1 || t[0].type != JSMN_OBJECT) {
    return -1;
  }

  /* Loop over all keys of the root object */
  for (i = 1; i < r; i++) {
    if (jsoneq(JSON_STRING, &t[i], "type") == 0) {
		jsmntok_t key = t[i+1];
		unsigned int length = key.end - key.start;
		memcpy(myType, &JSON_STRING[key.start], length);
		memset(myType+length, '\0', 1);
      i++;
    } else if (jsoneq(JSON_STRING, &t[i], "action") == 0) {
		jsmntok_t key = t[i+1];
		unsigned int length = key.end - key.start;
		memcpy(myAction, &JSON_STRING[key.start], length);
		memset(myAction+length, '\0', 1);
      i++;
    } else if (jsoneq(JSON_STRING, &t[i], "board") == 0) {
        jsmntok_t key = t[i+1];
        unsigned int length = key.end - key.start;
        memcpy(myBoard, &JSON_STRING[key.start], length);
        memset(myBoard+length, '\0', 1);
      i++;
    } else if (jsoneq(JSON_STRING, &t[i], "count") == 0) {
        jsmntok_t key = t[i+1];
        unsigned int length = key.end - key.start;
        memcpy(count_buff, &JSON_STRING[key.start], length);
        memset(count_buff+length, '\0', 1);
        *myCount = atoi( count_buff );
      i++;
    } 

  }
  return EXIT_SUCCESS;
}

//
//int main()
//{
//	char* myString = "{\"type\": \"shooter\", \"action\": motorOn}";
//	char* myType = malloc(sizeof(char)*10 + 1);
//	char* myAction = malloc(sizeof(char)*10 + 1);
//	memset(myType, '\0', sizeof(myType));
//	memset(myAction, '\0', sizeof(myAction));
//
//	parseAction(myString, myType, myAction);
//	printf("the type: ");
//	printf(myType);
//	printf("\nthe action: ");
//	printf(myAction);
//}
