#pragma once

#include <any>
#include <atomic>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <chrono>
#include <unordered_map>
#include <vector>
#include <stdexcept>

#include "phonebook.hpp"
#include "cpu_timer.hpp"

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
	 * [1]: https://en.wikipedia.org/wiki/Slab_allocation
	 * [2]: https://en.wikipedia.org/wiki/Multiple_buffering
	 *
	 *   - Asynchronous reading returns the most-recent event on the topic
	 * (idempotently). One can do this through (in any thread) the
	 * `ILLIXR::reader_latest` handle returned by `subscribe_latest()`.
	 *
	 *   - Synchronous reading schedules a callback to be executed on _every_ event
	 * which gets published. One can schedule computation by `schedule()`, which
	 * will run the computation in a thread managed by switchboard.
	 *
	 *
	 *     // Read topic 3 synchronously
	 *     sb->schedule<topic3_type>("task_1", "topic3", [&](switchboard::ptr<topic3_type>
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

		template <typename underlying_type>
		class event_wrapper : public event {
		private:
			underlying_type underlying_data;
		public:
			event_wrapper() { }
			event_wrapper(underlying_type underlying_data_)
				: underlying_data{underlying_data_}
			{ }
			operator underlying_type() const { return underlying_data; }
			underlying_type& operator*() { return underlying_data; }
			const underlying_type& operator*() const { return underlying_data; }
		};

	private:
	class topic {

	private:
		[[maybe_unused]] const std::string _m_name;
		const std::type_info& _m_ty;
		std::atomic<ptr<const event> *> _m_latest {nullptr};
		std::vector<std::function<void(ptr<const event>)>> _m_callbacks;
		std::mutex _m_callbacks_lock;
		queue<ptr<const event>> _m_queue {8 /*max size estimate*/};
		std::thread _m_thread;
		std::atomic<bool> _m_terminate {false};

		/* TODO: (optimization) use unique_ptr when there is only one
		   subscriber. */
		/* TODO: (optimization) use relaxed memory_order? */


	public:
		topic(const std::string& name, const std::type_info& ty)
			: _m_name{name}
			, _m_ty{ty}
		{ }
		topic(const topic&) = delete;
		topic& operator=(const topic&) = delete;

		const std::type_info& ty() { return _m_ty; }

		void process_callbacks() {
			while (!_m_terminate.load()) {
				ptr<const event> this_event;
				if (_m_queue.try_dequeue(this_event)) {
					const std::lock_guard<std::mutex> lock{_m_callbacks_lock};
					// std::cerr << "switchboard::topic::process_callbacks for " << this_event.get() << std::endl;
					for (const std::function<void(ptr<const event>)>& callback : _m_callbacks) {
						callback(this_event);
					}
				}
			}
		}

		~topic() {
			// std::cerr << "switchboard::topic::~topic" << std::endl;
			ptr<const event>* ev = _m_latest.exchange(nullptr);
			delete ev;
			// corresponds to new in most recent topic::writer::put or topic::topic (if put was never called)

			_m_terminate.store(true);
			if (_m_thread.joinable()) {
				// std::cerr << "switchboard::topic::~topic _m_terminate " << _m_terminate.load() << std::endl;
				_m_thread.join();
			}
			// "it is up to the user to ensure that the queue object is completely constructed before being used by any other threads"
			// - ConcurrentQueue documentation

			// draining the queue is necessary to reduce the ref-counts of the shared_ptrs in the queue, to reclaim memory.
			ptr<const event> elem;
			while (_m_queue.try_dequeue(elem)) {}
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
		void schedule(const std::string& account_name, std::function<void(ptr<const specific_event>)> callback) {
			const std::lock_guard<std::mutex> lock{_m_callbacks_lock};
			_m_callbacks.push_back([callback, &account_name](ptr<const event> this_event) {
				PRINT_CPU_TIME_FOR_THIS_BLOCK(account_name);
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
			void put(const specific_event* this_specific_event) {
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
				assert(this_event->use_count() == 1);
				// std::cerr << "swithcbord::writer::put allocated " << this_event << " for " << this_specific_event << std::endl;
				ptr<const event>* old_specific_event = _m_topic._m_latest.exchange(this_event);

				assert(_m_topic._m_queue.enqueue(*this_event));

				// pairs with the new above or from topic::topic (on the first time)
				// std::cerr << "swithcbord::writer::put decr ref-count for " << old_specific_event << std::endl;
				delete old_specific_event;
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
			bool valid() const {
				return bool{_m_topic._m_latest.load()};
			}

			ptr<const specific_event> get_latest_ro_nullable() const {
				/* Proof of thread-safety:
				   - Reads _m_topic, which is const.
				   - Reads _m_topic._m_latest using atomics.
				   - Increments the latest shared pointer before using it.
				     - Note that this is a const-pointer-to-data, so the pointer cannot be reseated, so there is no data-race on reading its value
					 - However, there is a data-race (both reader and writer using atomics) on the ref-count.
				*/
				ptr<const event>* this_event_ptr = _m_topic._m_latest.load();
				if (this_event_ptr) {
					ptr<const event> this_event = *this_event_ptr;
					ptr<const specific_event> this_specific_event = std::dynamic_pointer_cast<const specific_event>(this_event);
					// Check that the dynamic cast succeeded:
					// If the original (this_event) is non-null, then the child (this_specific_event) should be non-null
					assert(!this_event || this_specific_event);
					// this_event could stlil be null if the write rpushed null
					return this_specific_event;
				} else {
					// std::cerr << "switchboard::get_latest_ro_nullable: no event" << std::endl;
					return ptr<const specific_event>{nullptr};
				}
			}

			ptr<const specific_event> get_latest_ro() const {
				assert(valid());
				ptr<const specific_event> this_specific_event = get_latest_ro_nullable();
				assert(this_specific_event && "Writer pushed null, but reader is not reading nullable");
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
			_m_registry.try_emplace(name, name, typeid(specific_event));
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

		template <typename specific_event> void schedule(const std::string& account_name, const std::string& name, std::function<void(ptr<const specific_event>)> callback) {
			topic& this_topic = ensure_topic<specific_event>(name);
			// std::cerr << "registering callback " << callback << " on " << name << std::endl;
			this_topic.schedule(account_name, callback);
		}

	private:
		std::mutex _m_registry_lock;
		std::unordered_map<std::string, topic> _m_registry;
	};

	/* TODO: (usability) Do these HAVE to be smart pointers? If the
	   copy-constructor is already shallow, they could be concrete
	   data-types. */

} // namespace ILLIXR
