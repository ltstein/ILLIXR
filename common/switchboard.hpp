#pragma once

#include <any>
#include <atomic>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>
#include <stdexcept>

#include "phonebook.hpp"

#include "concurrentqueue.hpp"
template <typename T> using queue = moodycamel::ConcurrentQueue<T>;

namespace ILLIXR {

	/* TODO: (optimization:free-list) Use a free-list-managed shared_ptr. */
	/**
	 * @brief The type of shared pointer returned by switchboard.
	 */
	template <typename specific_event>
	using ptr = std::shared_ptr<specific_event>;

	// template <class event>
	// class pool_allocator
	// {
	// public:
	// 	[[nodiscard]] ptr<event>* allocate() {
	// 		_m_free_list_lock.lock();
	// 		if (free_list.empty()) {
	// 			_m_free_list_lock.unlock();
	// 			void* ptr = std::malloc(sizeof(event) + sizeof(ptr<event>));
	// 			event* this_event = new (ptr) event;
	// 			ptr += sizeof(event);
	// 			ptr<event>* this_ptr_event = new (ptr) ptr<event>{event};
	// 			return this_ptr_event;
	// 		} else {
	// 			ptr<event>* this_ptr_event = *_m_free_list.back();
	// 			_m_free_list.pop_back();
	// 			_m_free_list_lock.unlock();
	// 			return _this_ptr_event;
	// 		}
	// 	}
	// 	void deallocate(ptr<event>* this_ptr_event) noexcept {
	// 		_m_free_list_lock.lock();
	// 		if (_m_free_list.size() >= max_size) {
	// 			_m_free_list_lock.unlock();
	// 			void* ptr = std::static_cast<void*>(*this_ptr_event);
	// 			std::free(ptr);
	// 		} else {
	// 			_m_free_list.push_back(this_ptr_event);
	// 			_m_free_list_lock.unlock();
	// 		}
	// 	}
	// private:
	// 	std::vector<ptr<event>*> _m_free_list;
	// 	std::mutex _m_free_list_lock;
	// 	const std::size_t max_size = 16;
	// };

	/**
	 * @brief A manager for typesafe, threadsafe, named event-streams (called
	 * topics).
	 *
	 * - Writing: One can write to a topic (in any thread) through the
	 * `ILLIXR::writer` returned by `publish()`.
	 *
	 * - There are two ways of reading: asynchronous reading and synchronous
	 * reading:
	 *
	 *   - Asynchronous reading returns the most-recent event on the topic
	 * (idempotently). One can do this through (in any thread) the
	 * `ILLIXR::reader_latest` handle returned by `subscribe_latest()`.
	 *
	 *   - Synchronous reading schedules a callback to be executed on _every_ event
	 * which gets published. One can schedule computation by `schedule()`, which
	 * will run the computation in a thread managed by switchboard.
	 *
	 * \code{.cpp}
	 * void do_stuff(switchboard* sb) {
	 *     auto topic1 = sb->subscribe_latest<topic1_type>("topic1");
	 *     auto topic2 = sb->publish<topic2_type>("topic2");
	 *
	 *     // Read topic 3 synchronously
	 *     sb->schedule<topic3_type>("topic3", [&](switchboard::ptr<topic3_type>
	 * event3) {
	 *         // This is a lambda expression
	 *         // https://en.cppreference.com/w/cpp/language/lambda
	 *         std::cout << "Got a new event on topic3: " << event3->foo <<
	 * std::endl;
	 *     });
	 *
	 *     while (true) {
	 *         // Read topic 1
	 *         swirchboard::ptr<topic1_type> event1 = topic1.get_latest_ro();
	 *
	 *         // Write to topic 2
	 *         switchboard_ptr<topic2_type> event2 = topic2.allocate();
	 *         event2->foo = 3;
	 *         topic2.put(event2);
	 *     }
	 * }
	 * \endcode
	 *
	 */
	class switchboard : public phonebook::service {

	public:
		class event {
		public:
			virtual ~event() { }
		};

static void foo(ptr<const event> datum) {
	std::cerr << "switchboard: foo " << datum.get() << std::endl;
}

	private:
	class topic {

	private:
		[[maybe_unused]] const std::string _m_name;
		const std::type_info& _m_ty;
		std::atomic<ptr<const event> *> _m_latest {nullptr};
		std::vector<std::function<void(ptr<const event>)>> _m_callbacks;
		std::mutex _m_callbacks_lock;
		queue<ptr<const event>> _m_queue;
		std::thread _m_thread;
		const std::atomic<bool> &_m_terminate;

		/* TODO: (optimization) use unique_ptr when there is only one
		   subscriber. */
		/* TODO: (optimization) use relaxed memory_order? */


	public:
		topic(const std::string& name, const std::type_info& ty, const std::atomic<bool> &terminate)
			: _m_name{name}
			, _m_ty{ty}
			, _m_terminate{terminate}
		{ }
		topic(const topic&) = delete;
		topic& operator=(const topic&) = delete;

		const std::type_info& ty() { return _m_ty; }

		void process_callbacks() {
			while (!_m_terminate.load()) {
				ptr<const event> this_event;
				if (_m_queue.try_dequeue(this_event)) {
					const std::lock_guard<std::mutex> lock{_m_callbacks_lock};
					std::cerr << "switchboard::topic::process_callbacks for " << this_event.get() << std::endl;
					for (const std::function<void(ptr<const event>)>& callback : _m_callbacks) {
						callback(this_event);
					}
				}
			}
		}

		~topic() {
			ptr<const event>* ev = _m_latest.exchange(nullptr);
			delete ev;
			// corresponds to new in most recent topic::writer::put or topic::topic (if put was never called)
		}

		/**
		 * @brief Schedules the @p callback every time a new event is published.
		 *
		 * Switchboard maintains a threadpool to call `fn`. It is possible
		 * multiple instances of `fn` will be running concurrently if the
		 * event's repetition period is less than the runtime of `fn`.
		 *
		 * This is safe to be called from any thread.
		 *
		 * @throws if topic already exists, and its type does not match the `event`.
		 */
		template <typename specific_event>
		void schedule(std::function<void(ptr<const specific_event>)> callback) {
			const std::lock_guard<std::mutex> lock{_m_callbacks_lock};
			_m_callbacks.push_back([callback](ptr<const event> this_event) {
				assert(this_event);
				ptr<const specific_event> this_specific_event = std::dynamic_pointer_cast<const specific_event>(this_event);
				// assert dynamic cast succeeded
				// If this_event was non-null, then this_specific_event should be non-null
				assert(!this_event || this_specific_event);
				callback(this_specific_event);
			});

			if (!_m_thread.joinable()) {
				// Thread not yet started; start it now
				_m_thread = std::thread{std::bind(&topic::process_callbacks, this)};
				assert(_m_thread.joinable());
			}
		}

		/**
		 * @brief A handle for writing to this topic.
		 */
		template <typename specific_event>
		class writer {
		public:
			/**
			 * @brief Like `new`/`malloc` but more efficient for the specific case.
			 *
			 * There is an optimization available which has not yet been implemented:
			 * switchboard can memory from old events, like a [slab allocator][1].
			 * Suppose module A publishes data for module B. B's deallocation through
			 * the destructor, and A's allocation through this method completes the
			 * cycle in a [double-buffer (AKA swap-chain)][2].
			 *
			 * [1]: https://en.wikipedia.org/wiki/Slab_allocation
			 * [2]: https://en.wikipedia.org/wiki/Multiple_buffering
			 */
			specific_event* allocate() {
				specific_event* ret = new specific_event;
				assert(ret);
				return ret;
			}

			/**
			 * @brief Publish @p this_specific_event to this topic.
			 *
			 * @p this_specific_event must not be null, and it must not point to null data
			 *
			 */
			void put(specific_event* this_specific_event) {
				/*
				  Proof of thread-safety:
				  - Reads _m_topic, which is const.
				  - Modifies _m_topic._m_latest using atomics
				  - Modifies _m_topic._m_queue using concurrent primitives

				  One caveat:
				  While there is no data-race here, there is a synchronization race.
				  In the case where there are multiple writers to a topic,
				  A reader observing _m_latest could see events in a different order
				  than those observing _m_queue! However, I contend this is not a
				  problem, because I only guarantee that _m_topic._m_latest has
				  'sufficiently fresh data', so if 2 events come in at the same time, I
				  don't care which I publish to _m_latest. Also, there is not currently
				  any case where two threads write to the same topic in ILLIXR. I don't
				  want to acquire a lock here because it would be contended.
				*/
				assert(this_specific_event);
				// this new pairs with the delete below for, except for the last time, which pairs with the delete in topic::~topic
				ptr<const event>* this_event = new ptr<const event>{this_specific_event};
				std::cerr << "swithcbord::writer::put: allocated " << this_event << " for " << this_specific_event << std::endl;
				ptr<const event>* old_specific_event = _m_topic._m_latest.exchange(this_event);
				// pairs with the new above or from topic::topic (on the first time)
				std::cerr << "swithcbord::writer::put: decr ref-count for " << old_specific_event << std::endl;
				delete old_specific_event;

				assert(_m_topic._m_queue.enqueue(*this_event));
				assert(this_event->use_count() == 2);
			}

		private:
			friend class switchboard;
			writer(topic &topic) : _m_topic{topic} {}
			topic &_m_topic;
		};

		template <typename specific_event>
		class reader {
			/**
			 * @brief Gets a "read-only" copy of the latest value.
			 */

		public:
			ptr<const specific_event> get_latest_ro_nullable() {
				/* Proof of thread-safety:
				   - Reads _m_topic, which is const.
				   - Reads _m_topic._m_latest using atomics.
				*/
				ptr<const event>* this_event_ptr = _m_topic._m_latest.load();
				if (this_event_ptr) {
					std::cerr << "switchboard::get_latest_ro_nullable: " << this_event_ptr << std::endl;
					ptr<const event> this_event = *this_event_ptr;
					std::cerr << "switchboard::get_latest_ro_nullable: " << this_event_ptr << " " << this_event.get() << std::endl;
					ptr<const specific_event> this_specific_event = std::dynamic_pointer_cast<const specific_event>(this_event);
					// Check that the dynamic cast succeeded:
					// if this_event is non-null, then this_specific_event should also be non-null
					// This assertion is provable from the runtime typechecking I do in switchboard::get_reader/switchboard::get_writer (I hope)
					assert(!this_event || this_specific_event);
					return this_specific_event;
				} else {
					// no event is pushed yet
					return ptr<const specific_event>{nullptr};
				}
			}

			ptr<const specific_event> get_latest_ro() {
				ptr<const specific_event> this_specific_event = get_latest_ro_nullable();
				assert(this_specific_event);
				return this_specific_event;
			}

		private:
			friend class switchboard;
			reader(const topic &topic) : _m_topic{topic} {}
			const topic &_m_topic;
		};
	};

	public:

		/**
		 * @brief A handle which permits writing a topic
		 */
		template <typename specific_event> using writer = topic::writer<specific_event>;

		/**
		 * @brief A handle which permits reading a topic
		 */
		template <typename specific_event> using reader = topic::reader<specific_event>;


		template <typename specific_event> topic& ensure_topic(const std::string& name) {
			const std::lock_guard<std::mutex> lock{_m_registry_lock};
			_m_registry.try_emplace(name, name, typeid(specific_event), _m_terminate);
			topic& this_topic = _m_registry.at(name);
			assert(this_topic.ty() == typeid(specific_event));
			return this_topic;
		}

		template <typename specific_event> writer<specific_event> get_writer(const std::string& name) {
			topic& this_topic = ensure_topic<specific_event>(name);
			return writer<specific_event>(this_topic);
		}

		template <typename specific_event> reader<specific_event> get_reader(const std::string& name) {
			topic& this_topic = ensure_topic<specific_event>(name);
			return reader<specific_event>(this_topic);
		}

		template <typename specific_event> void schedule(const std::string& name, std::function<void(ptr<const specific_event>)> callback) {
			topic& this_topic = ensure_topic<specific_event>(name);
			// std::cerr << "registering callback " << callback << " on " << name << std::endl;
			this_topic.schedule(callback);
		}

	private:
		std::atomic<bool> _m_terminate{false};
		std::mutex _m_registry_lock;
		std::unordered_map<std::string, topic> _m_registry;
	};

	/* TODO: (usability) Do these HAVE to be smart pointers? If the
	   copy-constructor is already shallow, they could be concrete
	   data-types. */

} // namespace ILLIXR
