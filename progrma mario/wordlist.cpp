#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>

using namespace std;

const int INITIAL_LIST_CAPACITY = 10;

void build_wordlist (string* &word_list, int capacity, string filename)
{

    ifstream words;
    string word;
    int size = 0;

    // Allocate the space for the word list
    word_list = new string[INITIAL_LIST_CAPACITY];
    capacity = INITIAL_LIST_CAPACITY;

    // Open the file
    words.open (filename.c_str());

    if (!words) {
        cerr << "Could not open " << filename << endl;
        exit(EXIT_FAILURE);
    }

    // Loop to read in words
    words >> word;
    while (! words.eof()) {

        // If there isn't enough space, grow the array and copy the contents
        if (size > (capacity - 1)) {
            // Create another array that is twice as large as the word_list
            string *new_word_list = (new string[2 * capacity]);

            // Copy the words from the word_list into the new array
            for (int index = 0; (index < capacity); index++)
            {
                *new_word_list = word_list[index];
                cout << word_list[index] << endl;
            }

            delete word_list;
            word_list = new_word_list;

            capacity *= 2;
        }

        // Save the word
        word_list[size] = word;
        size++;

        // Read in next word
       words >> word;
        //cout << "holis" << endl;
    }

    words.close();
}

int main (int argc, char *argv[]) {

    // An array of string objects to store our
    // word list.

    string *word_list = 0;
    int capacity = 0;

    // Read the word list file
    build_wordlist (word_list, capacity, "words.txt");

    // Print out the words
    for (int index = 0; (index < capacity);
            index++) {
        cout << word_list[index] << endl;
    }

    return EXIT_SUCCESS;
}
