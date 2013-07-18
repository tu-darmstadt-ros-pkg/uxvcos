#include <options/options.h>
#include <system/FileStream.h>
#include <stream/ubx/Stream.h>
#include <system/Error.h>

int main(int argc, char **argv) {
  std::string filename;

  Options::add_options()("help,h", "This help.");
  Options::add_options()("log,l", Options::value<std::string>(&filename), "name of the logfile");
  Options::parse(argc, argv);

  if (Options::variables().count("help") || filename.empty()) {
    Options::help();
    return 0;
  }

  InStream *in = new System::InFileStream(filename);
  if (!in->open()) {
    std::cerr << "Could not open input file " << filename << ": " << System::Error::lastError() << std::endl;
    return -1;
  }

  UBX::Decoder *decoder = new UBX::Decoder(in);
  Data::Streamable *data;

  while(!in->eof()) {
    data = decoder->read();
    if (!Data::Streamable::isValid(data)) continue;
    std::cout << data->getTimestamp() << " " << data->getKey() << std::endl;
    delete data;
  }

  in->close();

  delete decoder;
  delete in;

  return 0;
}

