

#ifndef GNUGRAPH_H
#define GNUGRAPH_H


/*!
  @file gnugraph.h
  @brief Header file for graphics definitions.
*/

#ifdef _MSC_VER                  // Microsoft
#pragma warning (disable:4786)  // Disable decorated name truncation warnings 
#pragma warning (disable:4503)  // Disable decorated name truncation warnings 
#endif

#if defined(__WIN32__) || defined(_WIN32) || defined(__NT__)  || defined(__CYGWIN__)      /* Windows 95/NT */

#define GNUPLOT "wgnuplot_pipes -persist"
#define STRICT
#include <windows.h>

#ifdef _MSC_VER 
#define snprintf	_snprintf
#endif

#else // Unix 

//#define GNUPLOT "gnuplot"
#include <sys/types.h>
#include <unistd.h>
#endif

#include <stdio.h>
#include <stdexcept>

//#include <boost/shared_ptr.hpp>

#define WANT_STRING                  /* include.h will get string fns */
#define WANT_STREAM                  /* include.h will get stream fns */
#define WANT_FSTREAM                 /* include.h will get fstream fns */
#define WANT_MATH                    /* include.h will get math fns */
                                     /* newmatap.h will get include.h */
#include "newmatap.h"                /* need matrix applications */
#include "newmatio.h"                /* need matrix output routines */

#ifdef use_namespace
using namespace NEWMAT;
#endif

#include <sys/stat.h>

#include <sstream>
#include <vector>
//typedef boost::shared_ptr<GNUcurve> PSHR_Curve;
//typedef std::vector<PSHR_Curve> VectorCurves;

#define IO_COULD_NOT_OPEN_FILE  -1
#define IO_MISMATCH_SIZE        -2
#define IO_DATA_EMPTY           -3
#define IO_MISMATCH_ELEMENT_NBR -4
#define PROBLEM_FILE_READING    -5


/*!
  @class IO_matrix_file.
  @brief Read and write data at every iterations in a file.
*/
class IO_matrix_file {
public:
   IO_matrix_file(const std::string & filename);
   short write(const std::vector<Matrix> & data);
   short write(const std::vector<Matrix> & data, const std::vector<std::string> & title);
   short read(std::vector<Matrix> & data);
   short read(std::vector<Matrix> & data, std::vector<std::string> & title);
   short read_all(std::vector<Matrix> & data, std::vector<std::string> & data_title);
private:
   int 
     position_read,       //!< Position to read the file.
     nb_iterations_write, //!< Number of iterations in writing mode.
     nb_iterations_read,  //!< Number of iterations in reading mode.
     nb_element;          //!< Number of elements to read or write.
   std::string filename;       //!< File name.
};






#endif

