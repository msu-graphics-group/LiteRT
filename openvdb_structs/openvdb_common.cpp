#include "openvdb_common.h"

void
create_sdf4Mesh()
{
    openvdb::initialize();

    //  Create a VDB file object
    openvdb::io::File file("/home/james/Desktop/LiteRT/examples/models/armadillo.vdb");

    //  Open the file. This reads the file header, but not any grids
    file.open();

    // Loop over all grids in the file and retrieve a shared pointer
    // to the one named "LevelSetSphere".  (This can also be done
    // more simply by calling file.readGrid("LevelSetSphere").)
    openvdb::GridBase::Ptr baseGrid;

    for (openvdb::io::File::NameIterator nameIter = file.beginName(); nameIter != file.endName(); ++nameIter)
    {
        //  Read in only the grid we are interested in
        //  For us it is ls_armadillo

        if (nameIter.gridName() == "ls_armadillo")
        {
            std::cout << "Found " << nameIter.gridName() << std::endl;
            baseGrid = file.readGrid(nameIter.gridName());
        }
        else
        {
            std::cout << "skipping grid " << nameIter.gridName() << std::endl;
        }
    }

    file.close();

    // From the example above, "ls_armadillo" is known to be a FloatGrid,
    // so cast the generic grid pointer to a FloatGrid pointer.

    openvdb::FloatGrid::Ptr grid = openvdb::gridPtrCast<openvdb::FloatGrid>(baseGrid);
    std::cout << grid->tree().leafCount() << std::endl;
}

void 
load_mesh(const std::string& path)
{
    
}