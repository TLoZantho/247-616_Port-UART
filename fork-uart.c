#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <errno.h>

#define SERIAL_PORT "/dev/ttyUSB0"  // Adapté pour ton FTDI

// Initialisation du port série
int init_serial(const char *port) {
    int fd = open(port, O_RDWR | O_NOCTTY);
    if (fd == -1) {
        perror("Erreur ouverture port série");
        exit(EXIT_FAILURE);
    }

    struct termios options;
    if (tcgetattr(fd, &options) < 0) {
        perror("Erreur tcgetattr");
        close(fd);
        exit(EXIT_FAILURE);
    }

    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    options.c_cc[VMIN] = 2;    // Attendre 2 octets minimum
    options.c_cc[VTIME] = 0;   // Délai infini

    if (tcsetattr(fd, TCSANOW, &options) < 0) {
        perror("Erreur tcsetattr");
        close(fd);
        exit(EXIT_FAILURE);
    }

    return fd;
}

int main() {
    int fd = init_serial(SERIAL_PORT);
    pid_t pid = fork();

    if (pid < 0) {
        perror("Erreur fork");
        close(fd);
        exit(EXIT_FAILURE);
    }

    if (pid > 0) {
        // Processus Père : lecture
        printf("Je suis le processus Père, j'écrit sur la console (terminal) ce que j'entends sur le port série...\n");
        fflush(stdout);

        char buf[256];
        int total = 0;

        while (1) {
            int n = read(fd, buf + total, 1); // lire 1 octet
            if (n < 0) {
                perror("Erreur lecture");
                break;
            }
            if (n > 0) {
                total += n;
                // Dès que 2 octets sont reçus, ou fin de ligne, on affiche
                if (total >= 2 || buf[total - 1] == '\n') {
                    buf[total] = '\0';
                    printf("processus Père: nombres d'octets recus : %d --> %s\n", total, buf);
                    fflush(stdout);

                    if (strchr(buf, '!')) {
                        printf("Fin du Père\n");
                        fflush(stdout);
                        // Terminer le fils proprement
                        kill(pid, SIGTERM);
                        break;
                    }
                    total = 0;
                }
            }
        }
        wait(NULL); // Attendre le Fils
    } else {
        // Processus Fils : écriture
        printf("Je suis le processus Fils, j'écrit sur le port série ce que j'entends sur le terminal...\n");
        fflush(stdout);

        char input[256];

        while (1) {
            if (fgets(input, sizeof(input), stdin) == NULL) {
                perror("Erreur lecture stdin");
                break;
            }

            // Supprimer le '\n' de fgets
            input[strcspn(input, "\n")] = 0;

            int written = write(fd, input, strlen(input));
            if (written < 0) {
                perror("Erreur écriture");
                break;
            }

            tcdrain(fd); // Attendre que tout soit transmis
        }
    }

    close(fd);
    return 0;
}
