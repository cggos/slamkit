#include "util.hpp"

int main(int argc, char** argv) {
  bool bvoc = false;
  std::string str_voc_file;
#ifdef WITH_DBOW2
  str_voc_file = "voc.txt";
  ORBVocabulary vocab;
  bvoc = vocab.loadFromTextFile(str_voc_file);
#endif
#ifdef WITH_DBOW3
  str_voc_file = "voc.yml.gz";
  DBoW3::Vocabulary vocab(str_voc_file);
  bvoc = !vocab.empty();
#endif
  if (!bvoc) {
    cerr << "Wrong path to vocabulary. " << endl;
    cerr << "Falied to open at: " << str_voc_file << endl;
    exit(-1);
  }
  cout << "Vocabulary loaded!" << endl << endl;

  const std::vector<Mat>& descriptors = get_imgs(argv[1]);
  if (descriptors.empty()) return -1;

  cout << "comparing images with images " << endl;
  BowVector bow_vec0, bow_vec1;
  FeatureVector feat_vec0, feat_vec1;
  for (int i = 0; i < descriptors.size(); i++) {
#ifdef WITH_DBOW2
    // Feature vector associate features with nodes in the 4th level (from leaves up)
    // We assume the vocabulary tree has 6 levels, change the 4 otherwise
    vocab.transform(to_descriptor_vector(descriptors[i]), bow_vec0, feat_vec0, 4);
#endif
#ifdef WITH_DBOW3
    vocab.transform(descriptors[i], bow_vec0);
#endif
    for (int j = i; j < descriptors.size(); j++) {
#ifdef WITH_DBOW2
      vocab.transform(to_descriptor_vector(descriptors[j]), bow_vec1, feat_vec1, 4);
#endif
#ifdef WITH_DBOW3
      vocab.transform(descriptors[j], bow_vec1);
#endif
      double score = vocab.score(bow_vec0, bow_vec1);
      cout << "image " << i << " vs image " << j << " : " << score << endl;
    }
    cout << endl;
  }

#ifdef WITH_DBOW3
  cout << "comparing images with database " << endl;
  DBoW3::Database db(vocab, false, 0);
  for (int i = 0; i < descriptors.size(); i++) db.add(descriptors[i]);
  cout << "database info: " << db << endl;
  for (int i = 0; i < descriptors.size(); i++) {
    DBoW3::QueryResults ret;
    db.query(descriptors[i], ret, 4);  // max result=4
    cout << "searching for image " << i << " returns " << ret << endl << endl;
  }
#endif

  cout << "done." << endl;
}
