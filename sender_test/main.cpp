#include <unistd.h>
#include <vector>
#include <cstdlib>
#include <stdio.h>
#include <time.h>

int main(int argc, char* argv[]) {
    if (argc != 2) { // Vérification du nombre d'arguments
        fprintf(stderr, "Usage: %s <write pipe fd>\n", argv[0]); // Affichage d'un message d'erreur si le nombre d'arguments est incorrect
        exit(1); // Arrêt du programme
    }

    int write_fd = atoi(argv[1]); // Récupération du descripteur de fichier d'écriture du pipe à partir des arguments de la ligne de commande

    std::vector<float> numbers = {1.1, 2.2, 3.3, 4.4, 5.5}; // Déclaration du tableau de nombres à virgule flottante à envoyer
    write(write_fd, numbers.data(), numbers.size() * sizeof(float)); // Écriture des nombres dans le pipe
    sleep(3);
    while(1){
        fprintf(stderr,"Programm still running \n");
        sleep(1);
    }
    numbers = {6,5,4,32}; // Déclaration du tableau de nombres à virgule flottante à envoyer
    write(write_fd, numbers.data(), numbers.size() * sizeof(float)); // Écriture des nombres dans le pipe

    close(write_fd); // Fermeture du descripteur de fichier d'écriture du pipe

    return 0;
}
