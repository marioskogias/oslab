/* 
 * Performs a simple encryption-decryption 
 * of random data from /dev/urandom with the 
 * use of the cryptodev device.
 *
 * Stefanos Gerangelos <sgerag@cslab.ece.ntua.gr>
 * Vangelis Koukis <vkoukis@cslab.ece.ntua.gr>
 */

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <stddef.h>

#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>

#include <sys/types.h>
#include <sys/stat.h>

#include <sys/socket.h>

#include <arpa/inet.h>
#include <netinet/in.h>

#include <sys/un.h>
#include <signal.h>
#include "crypto/cryptodev.h"

#define DATA_SIZE       256
#define BLOCK_SIZE      16
#define KEY_SIZE        16  /* AES128 */

#define TCP_PORT    35001
#define TCP_BACKLOG 5


unsigned char res[DATA_SIZE];
unsigned char key[KEY_SIZE];
unsigned char iv[BLOCK_SIZE];
/* Insist until all of the data has been read */
ssize_t insist_read(int fd, void *buf, size_t cnt)
{
        ssize_t ret;
        size_t orig_cnt = cnt;

        while (cnt > 0) {
                ret = read(fd, buf, cnt);
                if (ret < 0)
                        return ret;
                buf += ret;
                cnt -= ret;
        }

        return orig_cnt;
}


ssize_t insist_write(int fd, const void *buf, size_t cnt)
{
        ssize_t ret;
        size_t orig_cnt = cnt;

        while (cnt > 0) {
                ret = write(fd, buf, cnt);
                if (ret < 0)
                        return ret;
                buf += ret;
                cnt -= ret;
        }

        return orig_cnt;
}

static int fill_urandom_buf(unsigned char *buf, size_t cnt)
{
        int crypto_fd;
        int ret = -1;

        crypto_fd = open("/dev/urandom", O_RDONLY);
        if (crypto_fd < 0)
                return crypto_fd;

        ret = insist_read(crypto_fd, buf, cnt);
        close(crypto_fd);

        return ret;
}

void init(int cfd,struct session_op * sess,unsigned char * key) {


        memset(sess, 0, sizeof(*sess));

        sess->cipher = CRYPTO_AES_CBC;
        sess->keylen = KEY_SIZE;
        sess->key = key;

        if (ioctl(cfd, CIOCGSESSION, sess)) {
                perror("ioctl(CIOCGSESSION)");
                exit(1);
        }
}
static int test_crypto(int cfd,unsigned char * p,int size,struct session_op * sess, struct crypt_op * cryp)
{
        int i = -1;
        struct {
                unsigned char   in[DATA_SIZE],
                                encrypted[DATA_SIZE],
                                decrypted[DATA_SIZE],
                                iv[BLOCK_SIZE],
                                key[KEY_SIZE];
        } data;


        memset(cryp, 0, sizeof(*cryp));
        memset(data.in,0,sizeof(data.in));
        memcpy(data.in,p,size);


        cryp->iv = iv;
        cryp->ses = sess->ses;
        cryp->src = data.in;
        cryp->dst = data.decrypted;
        cryp->op = COP_DECRYPT;
        cryp->len = sizeof(data.in);

        if (ioctl(cfd, CIOCCRYPT, cryp)) {
                perror("ioctl(CIOCCRYPT)");
                return 1;
        }


        memcpy(res,data.decrypted,DATA_SIZE);
        for(i=0;i<DATA_SIZE;i++)
                printf("%c",res[i]);
        return 0;
}

int main(int argv, char ** argc)
{
        int fd,cr;

        cr = open("/dev/crypto", O_RDWR);
        if (cr < 0) {
                perror("open(/dev/crypto)");
                return 1;
        }


        struct session_op sess;
        struct crypt_op cryp;


        memset(key,0,sizeof(key));
    //    fprintf(stdout,"Give the key\n");
      //  read(0,key,sizeof(key));
       // fprintf(stdout,"Give the iv\n");
        memset(iv,0,sizeof(iv));
       // read(0,iv,sizeof(iv));
        strcpy(key,"marios");
        strcpy(iv,"123");
        init(cr,&sess,key);
        char addrstr[INET_ADDRSTRLEN];
        int sd, newsd;
        ssize_t n;
        socklen_t len;
        struct sockaddr_un un;

        signal(SIGPIPE, SIG_IGN);

        if ((fd = socket(AF_UNIX, SOCK_STREAM, 0)) < 0) {
                perror("socket");
                exit(1);
        }
	unlink("decrypt.socket");
        fprintf(stderr, "Created Unix socket\n");

        memset(&un, 0, sizeof(un));
        un.sun_family = AF_UNIX;
        strcpy(un.sun_path,"decrypt.socket");
        int size = offsetof(struct sockaddr_un,sun_path) + strlen(un.sun_path) ;
        if (bind(fd, (struct sockaddr *)&un, size) < 0) {
                perror("bind");
                exit(1);
        }
        fprintf(stderr, "Bound unix socket\n");

        if (listen(fd, TCP_BACKLOG) < 0) {
                perror("listen");
                exit(1);
        }

        printf("accepted connection\n");

        if ((newsd = accept(fd, (struct sockaddr *)&un, &len)) < 0) {
                        perror("accept");
                        exit(1);
                }
       unsigned  char buf[DATA_SIZE];
        for (;;) {

                n = read(newsd, buf, sizeof(buf));
                if (n <= 0) {
                        if (n < 0)
                                perror("read from remote peer failed");
                        else
                                fprintf(stderr, "Peer went away\n");
                        break;
                }

                test_crypto(cr,buf,n,&sess,&cryp);
                if (insist_write(newsd, res, 256) != 256) {
                        perror("write to remote peer failed");
                        break;
                }
        }

        if (ioctl(cr, CIOCFSESSION, &sess.ses)) {
                perror("ioctl(CIOCFSESSION)");
                return 1;
        }

        if (close(newsd) < 0)
                perror("close");

        if (close(cr) < 0) {
                perror("close(fd)");
                return 1;
        }

        return 0;
}

