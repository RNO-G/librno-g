
extern "C" { 
#include "rno-g.h" 
#include "radiant.h" 
}

#include <pybind11/pybind11.h> 
#include <pybind11/pytypes.h> 
#include <pybind11/numpy.h> 
#include <pybind11/literals.h> 
#include <cstdlib> 

namespace py = pybind11;  

struct py_radiant 
{
    py_radiant(const std::string & spi, const std::string & uart, int trigger_gpio, int spi_enable) 
    {
      dev = radiant_open(spi.c_str(), uart.c_str(), trigger_gpio, spi_enable); 
      if (!dev) throw std::runtime_error("Could not open RADIANT"); 
    }

    ~py_radiant() { radiant_close(dev); } 
    radiant_dev_t * dev; 
}; 


PYBIND11_MODULE(radiant,m) 
{
   py::class_<py_radiant>(m,"Radiant") 
     .def(py::init<const std::string&,const std::string&,int,int>())
     .def("poll_trigger_ready", [](py_radiant & r, int timeout_ms) { return radiant_poll_trigger_ready(r.dev, timeout_ms); })
     .def("set_nbuffers_per_readout", [](py_radiant & r, int nbuf) { return radiant_set_nbuffers_per_readout(r.dev, nbuffers); })

     // not the best implementation... 
     .def("dump", [](py_radiant & r) { std::string ret(4096, '\0'); FILE *f = fmemopen(&ret[0], 4096, "rw"); radiant_dump(r.dev, f, 1); return ret; } )
     .def("set_run_number", [](py_radiant *r, int run) { radiant_set_run_number(r.dev, run); })
     .def("check_avail", [](py_radiant *r) { return radiant_check_avail(r.dev); })
     .def("labs_clear", [](py_radiant *r) { return radiant_labs_clear(r.dev); })
     .def("labs_start", [](py_radiant *r) { return radiant_labs_start(r.dev); })
     .def("labs_stop", [](py_radiant *r) { return radiant_labs_stop(r.dev); })
     .def("read_event", [](py_radiant *r, rno_g_header_t * hd, rno_g_waveorm_t *wf) { return radiant_read_event(r.dev,hd,wf); })
     .def("internal_trigger", [](py_radiant *r, int howmany, int block ) { return radiant_internal_trigger(r.dev,howmany,block); })
     .def("soft_trigger", [](py_radiant *r ) { return radiant_soft_trigger(r.dev); })
     .def("trigger_busy", [](py_radiant *r ) { int bsy, pending; int ret = radiant_trigger_busy(r.dev, &bsy, &pending); return py::dict("ret"_a=ret, , "bsy"_a=bsy, "clear_pending"_a=pending); })
       
         
   ;
} 
