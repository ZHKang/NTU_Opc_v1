#include "IOfile.h"

using namespace std;
IO_matrix_file::IO_matrix_file(const string & filename_)
	//!  @brief  Constructor.
{
	filename = filename_;
	position_read = 0;
	nb_iterations_write = 0;
	nb_iterations_read = 0;
	nb_element = 0;
}

short IO_matrix_file::write(const vector<Matrix> & data)
	//!  @brief Write data on disk using a default data name..
{
	vector<string> title;
	string tmp;
	for(unsigned int i = 1; i <= data.size(); i++)
	{
		tmp = "data#";  // Provide a default name
		tmp += i;
		title.push_back(tmp);
	}

	return IO_matrix_file::write(data, title);
}


short IO_matrix_file::write(const vector<Matrix> & data, const vector<string> & title)
/*!
  @brief Write data on disk.
  @param data: Data.
  @param title: Name of each data member (ie: speed, position, ...)
*/  
{
   /*
   If the file "filename" does not exist yet, created it. The first lines of the file
   contain the following informations (for each line):
      1) the number of iterations.
      2) second line is empty.
      3) the number(n) of matrix/iteration
      4) number of rows and number of columns of Matrix 1.
      5)  "                                         "   i.
      6)  "                                         "   n.
      7)---------------------------------   (end of header file)

   example of header file;
   1120
       
   2
   6 1 titre#1
   6 1 titre#1
   ---------------------------------
   */
   const char *ptr_filename = filename.c_str(); // transform string to *char
   if(data.size())
   {
      if(!nb_iterations_write)
      {
         struct stat buf;
         if(stat(ptr_filename, &buf) )  // File does not exist
         {
            ofstream outvecfile(ptr_filename);
            if(outvecfile)
            {
               outvecfile << "nd_iterations " << nb_iterations_write
               << "        " << endl;
               outvecfile << "nb_vector " << data.size() << endl;
               for(unsigned int i = 0; i < data.size(); i++)
                  outvecfile << "nb_rows " << data[i].Nrows() << "  "
                  << "nb_cols " << data[i].Ncols() <<  "  "
                  << title[i] << endl;
               outvecfile << "---------------------------------\n";
            }
            else
            {
               cerr << "IO_matrix_file::write: can not open file " << filename.c_str() << endl;
               return IO_COULD_NOT_OPEN_FILE;
            }
         }
         else
         {
            ifstream invecfile(ptr_filename, ios::in);
            if(invecfile)
               invecfile >> nb_iterations_write;
         }
      }

      ofstream outvecfile(ptr_filename, ios::in | ios::out);
      if(outvecfile)
      {
         outvecfile.seekp(strlen("nb_iterations ")); // position at start of fileObject
         outvecfile << ++nb_iterations_write << endl;
         outvecfile.seekp(0, std::ios::end); // position at end of fileObject
         for(unsigned int i = 0; i < data.size(); i++)
         {
            for(int j = 1; j <= data[i].Nrows(); j++) {
               for(int k = 1; k <= data[i].Ncols(); k++) {
                  outvecfile << data[i](j,k) << " ";
               }
            }
         }
         outvecfile << endl;
         outvecfile << endl;
      }
      else
      {
         cerr << "IO_matrix_file::write: can not open file " << filename.c_str() << endl;
         return IO_COULD_NOT_OPEN_FILE;
      }
   }
   else
   {
      cerr << "IO_matrix_file::write: vector data is empty" << endl;
      return IO_DATA_EMPTY;
   }

   return 0;
}