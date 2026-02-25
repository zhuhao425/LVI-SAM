#pragma once
// node_interface.h - Middleware-agnostic node interface for LVI-SAM.
//
// This header defines the abstract interface for the pub/sub system used by
// all LVI-SAM algorithmic components.  The interface is intentionally minimal:
// a Publisher<T> that can publish a message, and a NodeInterface that can
// subscribe to a topic and create publishers.
//
// USAGE - implementing a custom middleware adapter
// ------------------------------------------------
// Include your middleware headers (e.g., rclcpp, LCM, ZeroMQ…) and provide
// concrete types that satisfy the same API:
//
//   template<typename T>
//   class MyPublisher {
//   public:
//     void publish(const T& msg);
//     int  getNumSubscribers() const;
//     void shutdown();
//     bool isValid() const;
//   };
//
//   class MyNodeHandle {
//   public:
//     template<typename T>
//     MyPublisher<T> advertise(const std::string& topic, int queue_size);
//
//     template<typename MsgT, typename ClassT>
//     void subscribe(const std::string& topic, int queue_size,
//                    void (ClassT::*callback)(
//                        const std::shared_ptr<const MsgT>&),
//                    ClassT* obj);
//   };
//
// Then define LVISAMH_MIDDLEWARE_HEADER and the type aliases before including
// any LVI-SAM algorithm headers:
//
//   #define LVI_SAM_MIDDLEWARE_HEADER "my_middleware.h"
//   #include "my_middleware.h"
//   namespace ros { using NodeHandle = MyNodeHandle; ... }
//
// The algorithmic classes (ImageProjection, FeatureExtraction, …) are pure
// libraries: they contain no I/O code of their own.  A thin "node" wrapper
// (see each *_node.cpp) subscribes to topics, calls the algorithm, and
// publishes results.  Swapping the middleware only requires providing a
// compatible NodeHandle / Publisher implementation.

#include <functional>
#include <memory>
#include <string>

namespace lvi_sam {

// ---------------------------------------------------------------------------
// Callback type alias
// ---------------------------------------------------------------------------
template <typename T>
using MessageCallback = std::function<void(const std::shared_ptr<const T>&)>;

// ---------------------------------------------------------------------------
// Abstract publisher interface
// ---------------------------------------------------------------------------
template <typename T>
class PublisherInterface {
 public:
  virtual ~PublisherInterface() = default;
  virtual void Publish(const T& msg) = 0;
  virtual int  GetNumSubscribers() const = 0;
  virtual bool IsValid() const = 0;
};

// ---------------------------------------------------------------------------
// Abstract node interface
// Concrete implementations (standalone, ROS 1, ROS 2, …) derive from this.
// ---------------------------------------------------------------------------
class NodeInterface {
 public:
  virtual ~NodeInterface() = default;

  // Register a subscription.
  // The callback is invoked synchronously when a message is delivered to
  // the concrete middleware implementation.
  template <typename MsgT, typename ClassT>
  void Subscribe(const std::string& topic, int queue_size,
                 void (ClassT::*callback)(
                     const std::shared_ptr<const MsgT>&),
                 ClassT* obj);

  // Create a publisher for the given topic.
  template <typename T>
  std::shared_ptr<PublisherInterface<T>> Advertise(const std::string& topic,
                                                   int queue_size);

  // Read a parameter from the configuration back-end.
  template <typename T>
  bool GetParam(const std::string& key, T& value) const;
};

}  // namespace lvi_sam
