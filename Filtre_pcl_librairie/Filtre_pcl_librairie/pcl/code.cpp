#include <iostream>
#include <fstream>

//g++ -o code code.cpp

int main()
{
    // Ouvre le fichier en lecture
    std::ifstream file("input.txt");
    std::ofstream file2("output.txt");

    if (!file.is_open()) {
        std::cerr << "Erreur : impossible d'ouvrir le fichier !" << std::endl;
        return 1;
    }

    // Lit le fichier ligne par ligne
    std::string line;
    while (std::getline(file, line)) {
        // Recopie la ligne sauf les 2 premiers caractères
        std::string output = line.substr(2);
        // Affiche la ligne recopiée
        std::cout << output << std::endl;

        // on l'écrit dans le fichier
        
        file2 << output << std::endl;

    }

    // Ferme le fichier
    file.close();
    file2.close();

    return 0;
}

//g++ -o code code.cpp