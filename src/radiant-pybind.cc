
#include "rno-g.h" 
#include "radiant.h" 

#include <pybind11/pybind11.h> 
#include <pybind11/pytypes.h> 
#include <pybind11/numpy.h> 
#include <stdlib.h> 

namespace py = pybind11;  

struct py_radiant 
{
    py_radiant(const std::string & spi, const std::string & uart) 
    {
      dev = radiant_open(spi.c_str(), uart.c_str()); 
      if (!dev) throw std::runtime_error("Could not open RADIANT"); 
    }

    ~py_radiant() { radiant_close(dev); } 
    radiant_dev_t * dev; 
}; 


PYBIND11_MODULE(radiant,m) 
{
   py::class_<py_radiant>(m,"Radiant") 
     .def(py::init<const std::string&,const std::string&>())
     .def("set_run_number", []( py_radiant & r, int run)  { radiant_set_run_number(r.dev,run); })
     .def("clear", []( py_radiant & r)->int  { return radiant_clear(r.dev); })
     .def("reset", []( py_radiant & r)->int  { return radiant_reset(r.dev); })
     .def("rewind", []( py_radiant & r)->int  { return radiant_rewind(r.dev); })
     .def("set_read_mode", []( py_radiant & r, bool peek) ->void { radiant_set_read_mode(r.dev, peek); })
     .def("check_avail", [](py_radiant & r) ->int { return radiant_check_avail(r.dev); })
     .def("read_event", [](py_radiant & r, rno_g_header_t &hd, rno_g_waveform_t & wf) ->int { return radiant_read_event(r.dev, &hd, &wf); })
   ;

} 
