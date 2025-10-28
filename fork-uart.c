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
    SerialPortSettings.c_cc[VMIN] = 2;
    SerialPortSettings.c_cc[VTIME] = 0;

    // Appliquer les paramètres
    if ((tcsetattr(fd, TCSANOW, &SerialPortSettings)) != 0)
    {
        perror("Erreur configuration du port série");
        close(fd);
        exit(EXIT_FAILURE);
    }

    printf("Port série %s initialisé à 9600 8N1\n", device);
    return fd;
}


// -------------------------------------------------------
// Programme principal avec fork()
// -------------------------------------------------------
int main()
{
    int fd = initSerialPort(portTTY);
    pid_t pid = fork();

    if (pid < 0)
    {
        perror("Erreur fork()");
        close(fd);
        return EXIT_FAILURE;
    }

    if (pid == 0)
    {
        // ----------- Processus enfant -----------
        // Envoie ce que l’utilisateur tape vers le port série
        printf("Je suis le processus Fils, j'écrit sur le port série ce que j'entends sur la console (terminal)...\n");
        char c;
        printf("Enfant: Tapez des caractères à envoyer ('q' pour quitter)\n");

        while (1)
        {
            c = getchar(); // Lire depuis le terminal

            if (c == 'q')
            {
                printf("Enfant: sortie demandée.\n");
                break;
            }

            write(fd, &c, 1); // Envoyer sur port série
        }

        close(fd);
        exit(EXIT_SUCCESS);
    }
    else
    {
        // ----------- Processus parent -----------
        // Lit le port série et affiche à l’écran
        printf("Je suis le processus Père, j'écrit sur la console (terminal) ce que j'entends sur le port série...\n");
        char buffer[256];
        int n;
        printf("Parent: lecture du port série (arrêt sur '!')\n");

      while (1)
      {
        n = read(fd, buffer, sizeof(buffer) - 1); // lire jusqu'à 255 caractères
        if (n > 0)
        {
            buffer[n] = '\0'; // terminer la chaîne
            printf("processus Père: nombres d'octets reçus : %d --> %s\n", n, buffer);
            fflush(stdout);

            // Si '!' détecté dans le message, on termine
            if (strchr(buffer, '!') != NULL)
            {
                printf("Parent: '!' reçu, fin du programme.\n");
                kill(pid, SIGTERM);
                break;
            }
        }
      }
      close(fd);
      wait(NULL);
    }

  return 0;
}