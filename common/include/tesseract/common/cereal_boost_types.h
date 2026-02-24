#ifndef TESSERACT_COMMON_CEREAL_BOOST_TYPES_H
#define TESSERACT_COMMON_CEREAL_BOOST_TYPES_H

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/lexical_cast.hpp>

#include <cereal/cereal.hpp>

namespace cereal
{
template <class Archive>
void save(Archive& ar, const boost::uuids::uuid& g)
{
  const std::string data = boost::uuids::to_string(g);
  ar(CEREAL_NVP(data));
}

template <class Archive>
void load(Archive& ar, boost::uuids::uuid& g)
{
  std::string data;
  ar(CEREAL_NVP(data));
  g = boost::lexical_cast<boost::uuids::uuid>(data);
}
}  // namespace cereal

#endif  // TESSERACT_COMMON_CEREAL_BOOST_TYPES_H
