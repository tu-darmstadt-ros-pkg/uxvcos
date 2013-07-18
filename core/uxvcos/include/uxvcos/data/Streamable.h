//!  Streamable Class
/*!
  Base class for all streamable data objects used.
*/

#ifndef DATA_STREAMABLE_H
#define DATA_STREAMABLE_H

#include <stdlib.h>
#include <string>
#include <iostream>
#include <stdint.h>

#include <uxvcos/uxvcos.h>

#include <boost/array.hpp>
#include <vector>

#include <boost/type_traits.hpp>

#include <uxvcos/Time.h>
#include <uxvcos/data/Key.h>
#include <uxvcos/stream/Stream.h>

namespace Data {
using namespace uxvcos;

class TypeInfo;

template <typename Streamable, typename Archive>
static inline Archive& serialization(Streamable& data, Archive& ar);

template <typename Streamable, typename Archive>
static inline Archive& serialization(const Streamable& data, Archive& ar);

class UXVCOS_API Streamable {
public:
  static const size_t MAXSIZE = 2048;

  static Streamable * const INVALID;
  static Streamable * const BREAK;
  static bool isValid(Streamable *s) {
    return s != INVALID && s != BREAK;
  }

  Streamable() {}
  /*
  Streamable(const Streamable& other) {
    System::backtrace("WARNING! Copy constructor of class Streamable called!");
  }
  */
  virtual ~Streamable() {}

  /**
   * Streaming operator that reads from a stream.
   *
   * @param in The stream to read from.
   * @return The stream.
   */
  virtual InStream& operator<<(InStream& in) = 0; /*{
    return serialization(*this, in);
  }*/

  /**
   * Streaming operator that writes to a stream.
   *
   * @param out The stream to write to.
   * @return The stream.
   */
  virtual OutStream& operator>>(OutStream& out) const = 0; /*{
    return serialization(*this, out);
  }*/

	/**
	* Returns the name of this Streamable instance (defaults to the name of the type)
	*
	* @return name
	*/
	virtual std::string getName() const;

	/**
	* Returns the names of the fields this Streamable holds
	*
	* @return Array of Field Names terminated by a 0-field
	*/
	virtual const char* const* getFieldNames() const
	{
		return 0;
	}

	/**
	* Returns the names of the fields this Streamable holds (vector variant)
	*
	* @return Vector of field names
	*/
	typedef std::vector<std::string> FieldNames;
	virtual const FieldNames& getFields() const
	{
		static const FieldNames empty;
		return empty;
	}

	/**
	* Returns a prefix that shall be used to construct field names for unnamed fields
	*
	* @return Field name prefix
	*/
	virtual std::string getFieldPrefix() const
	{
		return "field";
	}

  /**
   * Retrieve the timestamp for the current data object.
   *
   * @return The timestamp.
   */
  virtual Time getTimestamp() const
  {
    return _timestamp;
  }

  /**
   * Set the timestamp for the current data object.
   *
   * @param The timestamp.
   */
  virtual void setTimestamp(Time timestamp = Time::now())
  {
    _timestamp = timestamp;
  }

  /**
   * Set the timestamp for the current data object.
   *
   * @param The timestamp.
   */
  virtual void setTimestamp(const Streamable& other)
  {
    _timestamp = other.getTimestamp();
  }

  /**
   * Retrieve the key for the current data object. Defaults to getTypeInfo()->getKey().
   *
   * @return The key.
   */
  virtual Key getKey() const;

  /**
   * Set the key for the current data object.
   *
   * @param The key.
   */
  virtual void setKey(Key key);

  virtual TypeInfo* getTypeInfo() const;

  virtual const std::string& getFormat() const
  {
    static const std::string emptyString;
    return emptyString;
  }

  virtual std::istream& operator<<(std::istream& is);
  virtual std::ostream& operator>>(std::ostream& os) const;

  template <typename T>
  T& as() { return dynamic_cast<T&>(*this); }

  template <typename T>
  const T& as() const { return dynamic_cast<const T&>(*this); }

  template <typename T>
  struct isStreamable : public boost::is_base_of<Streamable,T> {
    using boost::is_base_of<Streamable,T>::value;
    operator void *() { return static_cast<void *>(value); }
  };

protected:
  Time _timestamp;
  Key _key;
};

template <typename Streamable, typename Archive>
static inline Archive& serialization(Streamable& data, Archive& ar) {
  if (!data.serialize(ar)) ar.fail();
  return ar;
}

template <typename Streamable, typename Archive>
static inline Archive& serialization(const Streamable& data, Archive& ar) {
  if (!const_cast<Streamable&>(data).serialize(ar)) ar.fail();
  return ar;
}

/*
static inline OutStream& operator<<(OutStream& out, const Streamable& data) {
  return data >> out;
}

static inline InStream& operator>>(InStream& in, Streamable& data) {
  return data << in;
}
*/

static inline std::ostream& operator<<(std::ostream& os, const Streamable& data) {
  return data >> os;
}

static inline std::istream& operator>>(std::istream& is, Streamable& data) {
  return data << is;
}

} // namespace Data

#endif // DATA_STREAMABLE_H
