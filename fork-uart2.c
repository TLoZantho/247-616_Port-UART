#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>

const char *portTTY = "/dev/ttyUSB0"; // Adapter au bon port

// -------------------------------------------------------
// Fonction d'initialisation du port série
// -------------------------------------------------------
int initSerialPort(const char *device)
{
    int fd;
    struct termios SerialPortSettings;

    fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        perror("Erreur ouverture du port série");
        exit(EXIT_FAILURE);
    }

    // Effacer les flags O_NONBLOCK pour bloquer les lectures
    fcntl(fd, F_SETFL, 0);

    // Lecture des paramètres actuels
    tcgetattr(fd, &SerialPortSettings);

    // Configuration : 9600 8N1
    cfsetispeed(&SerialPortSettings, B9600);
    cfsetospeed(&SerialPortSettings, B9600);
    SerialPortSettings.c_cflag &= ~PARENB; // Pas de parité
    SerialPortSettings.c_cflag &= ~CSTOPB; // 1 stop bit
    SerialPortSettings.c_cflag &= ~CSIZE;
    SerialPortSettings.c_cflag |= CS8;     // 8 bits
    SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Activer réception, ignorer contrôle modem

    // Pas de contrôle de flux
    SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Mode raw : pas d’interprétation
    SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    SerialPortSettings.c_oflag &= ~OPOST;

    // Lecture bloquante pour 1 caractère (VMIN=1, VTIME=0)
    SerialPortSettings.c_cc[VMIN] = 0;
    SerialPortSettings.c_cc[VTIME] = 0;

    // Appliquer les paramètres
    if ((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0)
    {
        perror("Erreur configuration du port série");
        close(fd);
        exit(EXIT_FAILURE);
    }
    return fd;
}


// -------------------------------------------------------
// Programme principal avec fork()
// -------------------------------------------------------
int main()
{
    setvbuf(stdout, NULL, _IONBF, 0); // désactiver le buffering de stdout
    int fd = initSerialPort(portTTY);

    // ---- Fork pour le processus de lecture ----
    pid_t pidRead = fork();
    if (pidRead < 0)
    {
        perror("Erreur fork()");
        close(fd);
        return EXIT_FAILURE;
    }

    // ----------- Processus enfant Lecture -----------
    if (pidRead == 0)
    {
        printf("Enfant Lecture: Je lis le port série et affiche ce que je reçois...\n");
        char buffer[256];
        int n;
        while (1)
        {
            n = read(fd, buffer, sizeof(buffer) - 1);
            if (n > 0)
            {
                buffer[n] = '\0';
                printf("processus Lecture: nombres d'octets reçus : %d --> %s\n", n, buffer);
                fflush(stdout);

                if (strchr(buffer, '!') != NULL)
                {
                    printf("Lecture: '!' reçu, fin du processus Lecture\n");
                    break;
                }
            }
        }
        exit(EXIT_SUCCESS);
        close(fd);
    }


    // ---- Fork pour le processus d'écriture ----
    pid_t pidWrite = fork();
    if (pidWrite < 0)
    {
        perror("Erreur fork écriture");
        close(fd);
        exit(EXIT_FAILURE);
    }

    // ----------- Processus enfant Écriture -----------
    if (pidWrite == 0)
    {
        printf("Enfant Écriture: Je lis le terminal et envoie sur le port série...\n");
        char c;
        while (1)
        {
            c = getchar();
            if (c == 'q')
            {
                printf("Écriture: sortie demandée\n");
                break;
            }
            write(fd, &c, 1);
        }
        close(fd);
        exit(EXIT_SUCCESS);
    }


    // ----------- Processus principal actif -----------
    int n = 1;
    while (n < 10)
    {
        printf("Processus Principal: faire quelques trucs... (%d/10)\n", n);
        n++;
        sleep(3);
    }

    // Attendre la fin des enfants
    wait(NULL); // enfant Lecture
    wait(NULL); // enfant Écriture

    close(fd);
    printf("Fin du processus Principal\n");

    return 0;
}