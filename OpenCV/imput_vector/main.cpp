#include <iostream>
#include <vector>
#include <fstream>

using namespace std;

int main(int argc, char **argv) {
    if (argc < 1) {
        cout << "usage: pose_estimation_3d2d img1 img2 depth1 depth2" << endl;
        return 1;
    }
    ifstream infile;
    infile.open(argv[3]);
    if (!infile.is_open()) {
        cout << "please check out the path of file" << endl;
    }
    string s;
    vector<string> v1;
    while (getline(infile, s)) {
        v1.push_back(s);
    }
    for (int i = 0; i < v1.size(); i++) {
        cout << v1.at(i) << endl;
    }
    infile.close();
    return 0;
}
