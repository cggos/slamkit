#include <DBoW3/DBoW3.h>

#include "util.hpp"

int main(int argc, char** argv) {
  const std::vector<Mat>& descriptors = get_imgs(argv[1]);

  if (descriptors.empty()) return -1;

  cout << "creating vocabulary, please wait ... " << endl;
  DBoW3::Vocabulary vocab;
  vocab.create(descriptors);
  cout << "vocabulary info: " << vocab << endl;
  vocab.save("vocabulary.yml.gz");

  cout << "done" << endl;

  return 0;
}
