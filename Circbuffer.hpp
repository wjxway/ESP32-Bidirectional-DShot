/**
 * @file Circbuffer.hpp
 * @brief Library for circular buffer
 *
 * @note I intended to use boost::circular_buffer, but I cannot get it to work
 * with the compiler...
 *
 * @note This simple library is thread safe if
 * 1. the buffer size always larger than the content size (overriding in push
 * will never occur)
 * 2. there's only one writer (push) and one reader (pop).
 * when the previous condition cannot be satisfied, especially the first one,
 * just try to make sure that you don't peek too old messages
 *
 * @note this file also contain a copycat class. to use these two functions,
 * create a original class where only one task can add to it. then create
 * multiple copycats where you can pop(). each copycat act like a original, but
 * with no extra copy cost. and they won't interfere with each other. perfect
 * for duplicating streams of commands.
 */
#ifndef CIRCBUFFER_HPP__
#define CIRCBUFFER_HPP__

#include <cstdint>

// #define private public

/**
 * @brief copycat class for circbuffer
 *
 * @tparam T member type
 * @tparam msize length of circbuffer
 *
 * @note this class is a copycat for Circbuffer class. On the back of one
 * copycat class, there must be a original Circbuffer, copycat will not store
 * data, but points you to the original.
 *
 * You can only access data of the original with copycat, but cannot write. The
 * goal of copycat class is that you can instantiate a copy without cost, and
 * then use pop() to keep track of where you left off.
 *
 * @note when you call pop() in copycat class, you virtually pop out the
 * data by moving the head pointer, but will not actually pop out the data in
 * the original. This means you can have multiple copycats popping
 * independently. Actually, this is the whole point of creating this class, you
 * can have multiple 'streams' of commands of the same actual data without
 * interferening with each other, which is really helpful for multi-threading:
 * you can have a copycat class for the stream you want in each thread and just
 * use them as a stream (queue), without the actual cost of copying multiple
 * streams (queues).
 *
 * @note Because we need to deal with multi-threading here, it's not possible to
 * check for size() before peek() or pop(). and I do not want to add a flag to
 * indicate this unless absolutely necessary. So instead I will assume that this
 * function can only be used with msgs, and for msgs, their timing information
 * can not be 0. What you should do is: check for timing information after pop()
 * or peek() to determine whether they are valid. Yeah I know, I know, this is
 * ugly, but it makes the interface cleaner. This implementation also means that
 * you cannot use this class for general class T unless you don't care about
 * multi-threading.
 */
template <class T, uint32_t msize>
class Circbuffer_copycat;

/**
 * @brief circular buffer to deal with concurrency issues without the need of
 * lock, please just don't access near the element too deep inside the queue or
 * just don't do anything close to peek(max_size-1)
 *
 * @tparam T type
 * @tparam msize size of buffer
 *
 * @note not sure if we want to put msize here. it will instatiate a new class
 * for every size... seems redundant. but by giving the size inside template, we
 * can use plain arrays instead of vectors.
 */
template <class T, size_t msize>
class Circbuffer
{
public:
    /**
     * @brief Construct a new Circbuffer object
     * @param msize the maximum size of the buffer
     */
    Circbuffer() : max_size{msize}
    {
        n_elem = 0;

        end = start + msize - 1;
        tail = start;
        head = start;
    }
    /**
     * @brief push T into the tail of the queue
     *
     * @param cont content to push in
     */
    void push(const T &cont)
    {
        io_flag++;

        // push element
        *tail = cont;

        // increase tail
        tail = start + (tail + 1 - start) % max_size;

        // check if overlapped
        if (n_elem == max_size - 1)
        {
            head = start + (head + 1 - start) % max_size;
            head_rounds += (head == start);
        }
        else
        {
            n_elem++;
        }
    }
    /**
     * @brief pop an item out at the head and delete it
     * @warning not thread safe!
     * @note actually you shouldn't use it, you should make a copycat and pop()
     * on copycat class.
     *
     * @return T content
     */
    T pop()
    {
        io_flag++;

        T temp{};

        // if something inside, pop!
        if (n_elem)
        {
            temp = *head;

            head = start + (head + 1 - start) % max_size;
            head_rounds += (head == start);

            n_elem--;
        }

        return temp;
    }
    /**
     * @brief peek the item at the head without deleting it
     * @warning not thread safe!
     *
     * @return T content
     *
     * @note why would anyone use this?
     * @note return incorrect result when n_elem=0, be sure to check before
     * call.
     */
    T &peek()
    {
        return *head;
    }
    /**
     * @brief (not thread safe!) peek the index th oldest element.
     *
     * @param index
     * @return T content
     *
     * @note why would anyone use this?
     * @note return incorrect result when n_elem=0, be sure to check before
     * call.
     */
    T &peek(const size_t index)
    {
        return *(start + (head - start + index) % max_size);
    }

    /**
     * @brief (not thread safe!) peek the index th element counting from tail.
     *
     * @param index
     * @return T content
     *
     * @note return incorrect result when n_elem=0, be sure to check before
     * call.
     */
    T &peek_tail(const size_t index = 0)
    {
        return *(start + (tail - start + max_size - index - 1) % max_size);
    }

    /**
     * @brief clear all contents inside the buffer
     *
     * @note it won't actually clear everything, it only reset pointers!
     */
    void clear()
    {
        io_flag++;

        n_elem = 0;

        tail = start;
        head = start;

        // increment head_rounds so that when we clear up a buffer, the copycat will always update accordingly.
        head_rounds += 2U;
    }

    /**
     * @brief how many rounds has head move.
     *
     * @return uint32_t # of rounds
     */
    constexpr uint32_t Get_head_rounds() const
    {
        return head_rounds;
    }

    /**
     * @brief maximum size of the buffer
     */
    const size_t max_size;

    /**
     * @brief current number of elements
     *
     * @note make it volatile cause it could be changed by interrupts... though
     * not sure whether this is necessary.
     */
    volatile size_t n_elem;

    /**
     * @brief how many times has this been edited
     */
    uint32_t io_flag = 0;

    friend class Circbuffer_copycat<T, msize>;

private:
    /**
     * @brief start of buffer
     */
    T start[msize];
    /**
     * @brief end of buffer
     */
    T *end;

    /**
     * @brief current tail (where data is poped out)
     */
    T *tail;
    /**
     * @brief current head (where data is pushed in)
     */
    T *head;

    /**
     * @brief how many rounds has head rotate
     * @note every time head goes from end to start, head_rounds +1.
     */
    uint32_t head_rounds = 0;
};

template <class T, uint32_t msize>
class Circbuffer_copycat
{
public:
    /**
     * @brief Construct a new Circbuffer_copycat object
     *
     * @param orig1 a pointer to what it's gonna copy
     */
    Circbuffer_copycat(Circbuffer<T, msize> *orig1) : orig{orig1}, start{orig1->start}, end{orig1->end}, max_size{msize}
    {
        uint32_t curr_flag;
        do
        {
            curr_flag = orig->io_flag;
            head = orig->head;
            head_rounds = orig->head_rounds;
        } while (orig->io_flag != curr_flag);
    }

    /**
     * @brief (thread safe!) pop in the copycat class, note that this will only
     * change copycat's head pointer, but will not change the original class.
     *
     * @return T next element in buffer
     *
     * @note return T{} when n_elem=0, so when used with parsed_msgs, just check
     * the timing to determine whether the buffer is empty (if empty, time=0).
     * I know this is not elegant and using std::pair might be better, but hey I
     * want to keep the interface unified.
     */
    T pop()
    {
        T temp{};
        bool empty = false;
        bool not_first_time = false;

        do
        {
            // do not run this part for the first run
            if (not_first_time)
            {
                // reset head
                head = orig->head;
                head_rounds = orig->head_rounds;
            }
            else
            {
                not_first_time = true;
            }

            // it's not empty as long as head!=tail
            // even if concurrency happens here and make it true, the header must be invalid, so it will loop again.
            if (head != orig->tail)
            {
                empty = false;
                temp = *head;
            }
            // when empty, return T{}
            else
            {
                empty = true;
            }
        } while (head_invalid());

        // if not empty, increment head
        if (!empty)
        {
            head = start + (head + 1 - start) % max_size;
            head_rounds += (head == start);

            return temp;
        }

        return T{};
    }

    /**
     * @brief (thread safe!) peek the oldest.
     *
     * @return T
     *
     * @note return T{} when n_elem=0, so when used with parsed_msgs, just check the timing to determine whether the buffer is empty (if empty, time=0).
     *       I know this is not elegant and using std::pair might be better, but hey I want to keep the interface unified.
     */
    T peek()
    {
        T temp{};
        bool empty = false;
        bool not_first_time = false;

        do
        {
            // do not run this part for the first run
            if (not_first_time)
            {
                // reset head
                head = orig->head;
                head_rounds = orig->head_rounds;
            }
            else
            {
                not_first_time = true;
            }

            // it's not empty as long as head!=tail
            // even if concurrency happens here and make it true, the header must be invalid, so it will loop again.
            if (head != orig->tail)
            {
                empty = false;
                temp = *head;
            }
            // when empty, return T{}
            else
            {
                empty = true;
            }
        } while (head_invalid());

        return empty ? T{} : temp;
    }

    /**
     * @brief (thread safe!) check if empty.
     *
     * @return true for empty, false for populated
     */
    bool Empty_Q()
    {
        bool empty = false;
        bool not_first_time = false;

        do
        {
            // do not run this part for the first run
            if (not_first_time)
            {
                // reset head
                head = orig->head;
                head_rounds = orig->head_rounds;
            }
            else
            {
                not_first_time = true;
            }

            // it's not empty as long as head!=tail
            // even if concurrency happens here and make it true, the header must be invalid, so it will loop again.
            if (head != orig->tail)
            {
                empty = false;
            }
            else
            {
                empty = true;
            }
        } while (head_invalid());

        return empty;
    }

    /**
     * @brief how many rounds has the copycat's head go
     *
     * @return uint32_t rounds
     *
     * @note no idea why anyone want to use this
     */
    constexpr uint32_t Get_head_rounds() const
    {
        return head_rounds;
    }

    /**
     * @brief get io_flag of the original
     *
     * @return uint32_t io_flag
     */
    constexpr uint32_t Get_io_flags() const
    {
        return orig->io_flag;
    }

private:
    Circbuffer<T, msize> *orig;
    T *start;
    T *end;
    uint32_t max_size;

    /**
     * @brief head of copycat
     */
    T *head;
    /**
     * @brief head of copycat rounds
     */
    uint32_t head_rounds = 0;

    /**
     * @brief check if head of copycat is valid
     *
     * @note it's invalid when: (head_rounds==orig->head_rounds&&head<orig->head)||(head_rounds<orig->head_rounds)
     */
    constexpr bool head_invalid() const
    {
        return (head_rounds == orig->head_rounds && head < orig->head) || (head_rounds < orig->head_rounds);
    }
};

#endif