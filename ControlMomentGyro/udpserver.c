#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <netdb.h>
#include <sys/time.h>

const int buf_size = 256;  /* buffer size for the udp packets */
const int portno   = 8691; /* port to listen on */

unsigned int       shared_clientlen = 0;  /* byte size of client's address */
struct sockaddr_in shared_clientaddr;     /* client addr */
pthread_mutex_t    mutex;                 /* mutex used to send client address to the send_msg thread */

/*
 * error - wrapper for perror
 */
void error(char *msg) {
    perror(msg);
    exit(1);
}

void *send_msg(void *sock_desc) {
    char buf[buf_size];            /* message buf */
    int clientlen;                 /* byte size of client's address */
    struct sockaddr_in clientaddr; /* client addr */
    int msgsize, nsent;                         /* message byte size */
    struct timeval stime, etime;
    int elapsed;

    gettimeofday(&stime, NULL);

    while (1) {
        gettimeofday(&etime, NULL);
        elapsed = ((etime.tv_sec - stime.tv_sec) * 1000000) + (etime.tv_usec - stime.tv_usec);
        if (elapsed<160) continue;
        stime = etime;

        pthread_mutex_lock(&mutex);
        clientaddr = shared_clientaddr;
        clientlen  = shared_clientlen;
        pthread_mutex_unlock(&mutex);

        if (!clientlen) continue;

        msgsize = sizeof(struct timeval);
        memcpy(buf, &stime, msgsize);
//      sprintf(buf, "sdfsdf");
//      msgsize = strlen(buf);

        nsent = sendto(*(int *)sock_desc, buf, msgsize, 0, (struct sockaddr *)&clientaddr, clientlen);
//        printf("sendto: %ld %d\n",clientaddr.sin_addr.s_addr, clientlen);
        if (nsent < 0) {
            error("ERROR in sendto");
        }
    }
}

void *receive_msg(void *sock_desc) {
    char buf[buf_size];            /* message buf */
    int n;                         /* message byte size */
    unsigned int clientlen;        /* byte size of client's address */
    struct sockaddr_in clientaddr; /* client addr */
    struct timeval stime, etime, ttime;
    int elapsed;

    gettimeofday(&stime, NULL);

    while (1) {
        gettimeofday(&etime, NULL);
        elapsed = ((etime.tv_sec - stime.tv_sec) * 1000000) + (etime.tv_usec - stime.tv_usec);
        stime = etime;

        memset((void *)buf, 0, buf_size);
        n = recvfrom(*(int *)sock_desc, buf, buf_size, 0, (struct sockaddr *)&clientaddr, &clientlen);
        if (n <= 0) {
            error("ERROR in recvfrom");
        }
        if (!n || !clientaddr.sin_port || !clientaddr.sin_addr.s_addr) continue;

        pthread_mutex_lock(&mutex);
        shared_clientaddr = clientaddr;
        shared_clientlen  = clientlen;
        pthread_mutex_unlock(&mutex);

//        printf("server received %d/%d bytes: %s\n", (int)strlen(buf), n, buf);
//        printf("server received %d bytes\n", n);
        printf("time between two messages: %d\n", elapsed);
        if (n==sizeof(struct timeval)) {
            gettimeofday(&etime, NULL);
            memcpy(&ttime, buf, n);
            elapsed = ((etime.tv_sec - ttime.tv_sec) * 1000000) + (etime.tv_usec - ttime.tv_usec);
            printf("send-return time: %d\n", elapsed);
        }
    }
}

int main() {
    int sockfd;                    /* socket */
    struct sockaddr_in serveraddr; /* server's addr */
    int optval;                    /* flag value for setsockopt */

    if (pthread_mutex_init(&mutex, NULL) != 0) {
        error("ERROR in mutex init\n");
    }

    /*
     * socket: create the parent socket
     */
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        error("ERROR opening socket");
    }

    /* setsockopt: Handy debugging trick that lets
     * us rerun the server immediately after we kill it;
     * otherwise we have to wait about 20 secs.
     * Eliminates "ERROR on binding: Address already in use" error.
     */
    optval = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval , sizeof(int));

    /*
     * build the server's Internet address
     */
    memset((void *)&serveraddr, 0, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serveraddr.sin_port = htons((unsigned short)portno);

    /*
     * bind: associate the parent socket with a port
     */
    if (bind(sockfd, (struct sockaddr *)&serveraddr, sizeof(serveraddr)) < 0) {
        error("ERROR on binding");
    }

    pthread_t send_thread, receive_thread;
    pthread_create(&send_thread,    NULL, send_msg,    (void *) &sockfd);
    pthread_create(&receive_thread, NULL, receive_msg, (void *) &sockfd);

    pthread_join(receive_thread, NULL);
    pthread_join(send_thread,    NULL);

    pthread_mutex_destroy(&mutex);

    exit(EXIT_SUCCESS);
}

