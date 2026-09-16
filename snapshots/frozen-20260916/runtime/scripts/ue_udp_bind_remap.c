#define _GNU_SOURCE

#include <arpa/inet.h>
#include <dlfcn.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <sys/socket.h>

/* Keep UDP 9999 for OLI and remap duplicate packaged-renderer binds. */
static atomic_uint target_bind_count = 0;

int bind(int fd, const struct sockaddr *address, socklen_t length) {
    static int (*real_bind)(int, const struct sockaddr *, socklen_t) = NULL;
    if (real_bind == NULL) {
        real_bind = dlsym(RTLD_NEXT, "bind");
    }

    const char *enabled = getenv("MATRIX_REMAP_DUP_UDP_9999");
    if (enabled != NULL && enabled[0] == '1' && address != NULL &&
        address->sa_family == AF_INET &&
        length >= (socklen_t)sizeof(struct sockaddr_in)) {
        const struct sockaddr_in *original = (const struct sockaddr_in *)address;
        if (ntohs(original->sin_port) == 9999) {
            unsigned int index = atomic_fetch_add(&target_bind_count, 1);
            if (index > 0) {
                struct sockaddr_in remapped = *original;
                remapped.sin_port = htons(19999);
                return real_bind(fd, (const struct sockaddr *)&remapped,
                                 sizeof(remapped));
            }
        }
    }
    return real_bind(fd, address, length);
}
