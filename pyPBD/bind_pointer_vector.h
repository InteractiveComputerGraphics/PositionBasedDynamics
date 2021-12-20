//
// Created by sjeske on 2/11/20.
//

#pragma once

#include <pybind11/detail/common.h>
#include <pybind11/operators.h>

#include <algorithm>
#include <sstream>
#include <type_traits>

PYBIND11_NAMESPACE_BEGIN(PYBIND11_NAMESPACE)
PYBIND11_NAMESPACE_BEGIN(detail)

// Vector modifiers -- requires a copyable vector_type:
// (Technically, some of these (pop and __delitem__) don't actually require copyability, but it seems
// silly to allow deletion but not insertion, so include them here too.)
template <typename Vector, typename Class_>
void vector_pointer_modifiers(enable_if_t<is_copy_constructible<typename Vector::value_type>::value, Class_> &cl) {
        using T = typename Vector::value_type;
        using TNoPtr = typename std::remove_pointer<T>::type;
        using SizeType = typename Vector::size_type;
        using DiffType = typename Vector::difference_type;

        auto wrap_i = [](DiffType i, SizeType n) {
            if (i < 0)
                i += n;
            if (i < 0 || (SizeType)i >= n)
                throw index_error();
            return i;
        };

        cl.def("append",
               [](Vector &v, const T &value) {
                v.push_back(new TNoPtr(*value));
            },
               arg("x"),
               "Add an item to the end of the list");

        cl.def(init([](iterable it) {
            auto v = std::unique_ptr<Vector>(new Vector());
            v->reserve(len_hint(it));
            for (handle h : it)
                v->push_back(new TNoPtr(*h.cast<T>()));
            return v.release();
        }));

        cl.def("clear",
               [](Vector &v) {
                    for(const auto & elem : v)
                        delete elem;
                   v.clear();
               },
               "Clear the contents"
        );

        cl.def("extend",
               [](Vector &v, const Vector &src) {
                   for(const auto & elem : src)
                       v.push_back(new TNoPtr(*elem));
               },
               arg("L"),
               "Extend the list by appending all the items in the given list"
        );

        cl.def("extend",
               [](Vector &v, iterable it) {
                   const size_t old_size = v.size();
                   v.reserve(old_size + len_hint(it));
                   try {
                       for (handle h : it) {
                           v.push_back(new TNoPtr(*h.cast<T>()));
                       }
                   } catch (const cast_error &) {
                       for(auto it = v.begin() + static_cast<DiffType>(old_size); it != v.end(); it++)
                           delete *it;
                       v.erase(v.begin() + static_cast<typename Vector::difference_type>(old_size), v.end());
                       try {
                           v.shrink_to_fit();
                       } catch (const std::exception &) {
                           // Do nothing
                       }
                       throw;
                   }
               },
               arg("L"),
               "Extend the list by appending all the items in the given list"
        );

        cl.def("insert",
               [](Vector &v, DiffType i, const T &x) {
                   // Can't use wrap_i; i == v.size() is OK
                   if (i < 0)
                       i += v.size();
                   if (i < 0 || (SizeType)i > v.size())
                       throw index_error();
                   v.insert(v.begin() + i, new TNoPtr(*x));
               },
               arg("i") , arg("x"),
               "Insert an item at a given position."
        );

        cl.def("pop",
               [](Vector &v) {
                   if (v.empty())
                       throw index_error();
                   T t = new TNoPtr(*v.back());
                   delete v.back();
                   v.pop_back();
                   return t;
               },
               "Remove and return the last item"
        );

        cl.def("pop",
               [wrap_i](Vector &v, DiffType i) {
                   i = wrap_i(i, v.size());
                   T t = new TNoPtr(*v[(SizeType) i]);
                   delete v[(SizeType) i];
                   v.erase(v.begin() + i);
                   return t;
               },
               arg("i"),
               "Remove and return the item at index ``i``"
        );

        cl.def("__setitem__",
               [wrap_i](Vector &v, DiffType i, const T &t) {
                   i = wrap_i(i, v.size());
                   *v[(SizeType)i] = *t;
               }
        );

        /// Slicing protocol
        cl.def("__getitem__",
               [](const Vector &v, slice slice) -> Vector * {
                   size_t start, stop, step, slicelength;

                   if (!slice.compute(v.size(), &start, &stop, &step, &slicelength))
                       throw error_already_set();

                   Vector *seq = new Vector();
                   seq->reserve((size_t) slicelength);

                   for (size_t i=0; i<slicelength; ++i) {
                       seq->push_back(v[start]);    // TODO: Memory management still taken care of by C++
                       start += step;
                   }
                   return seq;
               },
               arg("s"),
               "Retrieve list elements using a slice object"
        );

        cl.def("__setitem__",
               [](Vector &v, slice slice,  const Vector &value) {
                   size_t start, stop, step, slicelength;
                   if (!slice.compute(v.size(), &start, &stop, &step, &slicelength))
                       throw error_already_set();

                   if (slicelength != value.size())
                       throw std::runtime_error("Left and right hand size of slice assignment have different sizes!");

                   for (size_t i=0; i<slicelength; ++i) {
                       *v[start] = *value[i];
                       start += step;
                   }
               },
               "Assign list elements using a slice object"
        );

        cl.def("__delitem__",
               [wrap_i](Vector &v, DiffType i) {
                   i = wrap_i(i, v.size());
                   delete v[(SizeType) i];
                   v.erase(v.begin() + i);
               },
               "Delete the list elements at index ``i``"
        );

        cl.def("__delitem__",
               [](Vector &v, slice slice) {
                   size_t start, stop, step, slicelength;

                   if (!slice.compute(v.size(), &start, &stop, &step, &slicelength))
                       throw error_already_set();

                   if (step == 1 && false) {
                       v.erase(v.begin() + (DiffType) start, v.begin() + DiffType(start + slicelength));
                   } else {
                       for (size_t i = 0; i < slicelength; ++i) {
                           delete v[(SizeType) start];
                           v.erase(v.begin() + DiffType(start));
                           start += step - 1;
                       }
                   }
               },
               "Delete list elements using a slice object"
        );

    }
PYBIND11_NAMESPACE_END(detail)
//
// std::vector of raw pointers
//
template <typename Vector, typename holder_type = std::unique_ptr<Vector>, typename... Args>
class_<Vector, holder_type> bind_pointer_vector(handle scope, std::string const &name, Args&&... args) {
        using Class_ = class_<Vector, holder_type>;

        // If the value_type is unregistered (e.g. a converting type) or is itself registered
        // module-local then make the vector binding module-local as well:
        using vtype = typename Vector::value_type;
        auto vtype_info = detail::get_type_info(typeid(vtype));
        bool local = !vtype_info || vtype_info->module_local;

        Class_ cl(scope, name.c_str(), pybind11::module_local(local), std::forward<Args>(args)...);

        // Declare the buffer interface if a buffer_protocol() is passed in
        detail::vector_buffer<Vector, Class_, Args...>(cl);

        cl.def(init<>());

        // Register copy constructor (if possible)
        detail::vector_if_copy_constructible<Vector, Class_>(cl);

        // Register comparison-related operators and functions (if possible)
        detail::vector_if_equal_operator<Vector, Class_>(cl);

        // Register stream insertion operator (if possible)
        detail::vector_if_insertion_operator<Vector, Class_>(cl, name);

        // Modifiers require copyable vector value type
        detail::vector_pointer_modifiers<Vector, Class_>(cl);

        // Accessor and iterator; return by value if copyable, otherwise we return by ref + keep-alive
        detail::vector_accessor<Vector, Class_>(cl);

        cl.def("__bool__",
               [](const Vector &v) -> bool {
                   return !v.empty();
               },
               "Check whether the list is nonempty"
        );

        cl.def("__len__", &Vector::size);

        return cl;
    }

PYBIND11_NAMESPACE_END(PYBIND11_NAMESPACE)
