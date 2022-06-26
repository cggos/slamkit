#include <DBoW3/DBoW3.h>

#include "util.hpp"

int main(int argc, char** argv) {
  DBoW3::Vocabulary vocab("./vocabulary.yml.gz");
  if (vocab.empty()) {
    cerr << "Vocabulary does not exist." << endl;
    return 1;
  }

  const std::vector<Mat>& descriptors = get_imgs(argv[1]);
  if (descriptors.empty()) return -1;

  cout << "comparing images with images " << endl;
  for (int i = 0; i < descriptors.size(); i++) {
    DBoW3::BowVector v1;
    vocab.transform(descriptors[i], v1);
    for (int j = i; j < descriptors.size(); j++) {
      DBoW3::BowVector v2;
      vocab.transform(descriptors[j], v2);
      double score = vocab.score(v1, v2);
      cout << "image " << i << " vs image " << j << " : " << score << endl;
    }
    cout << endl;
  }

  cout << "comparing images with database " << endl;
  DBoW3::Database db(vocab, false, 0);
  for (int i = 0; i < descriptors.size(); i++) db.add(descriptors[i]);
  cout << "database info: " << db << endl;
  for (int i = 0; i < descriptors.size(); i++) {
    DBoW3::QueryResults ret;
    db.query(descriptors[i], ret, 4);  // max result=4
    cout << "searching for image " << i << " returns " << ret << endl << endl;
  }
  cout << "done." << endl;
}
