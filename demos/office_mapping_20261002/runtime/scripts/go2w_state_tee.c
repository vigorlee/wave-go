/* Read-only telemetry: copy this simulator's localhost:9999 state datagrams
 * to localhost:25011. The original send, destination and result are unchanged.
 * Loaded only into robot_mujoco for the extended scene, never into UE/ROS. */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <dlfcn.h>
#include <errno.h>
#include <pthread.h>
#include <sys/socket.h>

static ssize_t (*original_sendto)(int, const void *, size_t, int,
                                 const struct sockaddr *, socklen_t);
static pthread_once_t init_once = PTHREAD_ONCE_INIT;
static void initialize(void) {
    original_sendto = dlsym(RTLD_NEXT, "sendto");
}

ssize_t sendto(int fd, const void *buf, size_t size, int flags,
               const struct sockaddr *address, socklen_t address_size) {
    pthread_once(&init_once, initialize);
    if (!original_sendto) { errno = ENOSYS; return -1; }
    ssize_t result = original_sendto(fd, buf, size, flags, address, address_size);
    int saved_errno = errno;
    if (result >= 0 && address && address_size >= sizeof(struct sockaddr_in)
        && address->sa_family == AF_INET) {
        const struct sockaddr_in *target = (const struct sockaddr_in *)address;
        if (ntohs(target->sin_port) == 9999
            && ntohl(target->sin_addr.s_addr) == INADDR_LOOPBACK) {
            struct sockaddr_in copy = *target;
            copy.sin_port = htons(25011);
            original_sendto(fd, buf, size, flags | MSG_DONTWAIT,
                            (const struct sockaddr *)&copy, sizeof(copy));
        }
    }
    errno = saved_errno;
    return result;
}
