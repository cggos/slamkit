#include "util.hpp"

int main(int argc, char** argv) {
  const std::vector<Mat>& descriptors = get_imgs(argv[1]);

  if (descriptors.empty()) return -1;

#ifdef WITH_DBOW2
  cout << "creating vocabulary with DBoW2, please wait ... " << endl;
  ORBVocabulary vocab;
  std::vector<std::vector<cv::Mat>> vv_descs;
  vv_descs.resize(descriptors.size());
  for (int i = 0; i < vv_descs.size(); i++) vv_descs[i] = to_descriptor_vector(descriptors[i]);
  vocab.create(vv_descs);
  vocab.saveToTextFile("voc.txt");
#endif
#ifdef WITH_DBOW3
  cout << "creating vocabulary with DBoW3, please wait ... " << endl;
  DBoW3::Vocabulary vocab;
  vocab.create(descriptors);
  vocab.save("voc.yml.gz");
#endif
  cout << "vocabulary info: " << vocab << endl;

  cout << "done" << endl;

  return 0;
}
