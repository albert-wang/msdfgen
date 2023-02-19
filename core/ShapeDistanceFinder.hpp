
#include "ShapeDistanceFinder.h"

namespace msdfgen
{
	template<class ContourCombiner>
	ShapeDistanceFinder<ContourCombiner>::ShapeDistanceFinder(const Shape& shape)
		: shape(shape)
		, contourCombiner(shape)
		, shapeEdgeCache(shape.edgeCount())
	{
	}

	template<class ContourCombiner>
	typename ShapeDistanceFinder<ContourCombiner>::DistanceType ShapeDistanceFinder<ContourCombiner>::distance(const Point2& origin)
	{
		contourCombiner.reset(origin);
		typename ContourCombiner::EdgeSelectorType::EdgeCache* edgeCache = &shapeEdgeCache[0];

		for (size_t i = 0; i < shape.contours.size(); ++i)
		{
			const Contour* contour = &shape.contours[i];
			if (!contour->edges.empty())
			{
				typename ContourCombiner::EdgeSelectorType& edgeSelector = contourCombiner.edgeSelector(int(i));

				const EdgeSegment* prevEdge = contour->edges.size() >= 2 ?
					contour->edges[contour->edges.size() - 2] : // One before end
					contour->edges[0];

				const EdgeSegment* curEdge = contour->edges.back();

				for (size_t j = 0; j < contour->edges.size(); ++j)
				{
					const EdgeSegment* nextEdge = contour->edges[j];
					edgeSelector.addEdge(*edgeCache++, prevEdge, curEdge, nextEdge);
					prevEdge = curEdge;
					curEdge = nextEdge;
				}
			}
		}

		return contourCombiner.distance();
	}

	template<class ContourCombiner>
	typename ShapeDistanceFinder<ContourCombiner>::DistanceType ShapeDistanceFinder<ContourCombiner>::oneShotDistance(const Shape& shape, const Point2& origin)
	{
		ContourCombiner contourCombiner(shape);
		contourCombiner.reset(origin);

		for (size_t i = 0; i < shape.contours.size(); ++i)
		{
			const Contour* contour = &shape.contours[i];
			if (!contour->edges.empty())
			{
				typename ContourCombiner::EdgeSelectorType& edgeSelector = contourCombiner.edgeSelector(int(i));

				const EdgeSegment* prevEdge = contour->edges.size() >= 2 ?
					contour->edges[contour->edges.size() - 2] : // One before end
					contour->edges[0];

				const EdgeSegment* curEdge = contour->edges.back();
				
				for (size_t j = 0; j < contour->edges.size(); ++j)
				{
					const EdgeSegment* nextEdge = contour->edges[j];
					typename ContourCombiner::EdgeSelectorType::EdgeCache dummy;
					edgeSelector.addEdge(dummy, prevEdge, curEdge, nextEdge);
					prevEdge = curEdge;
					curEdge = nextEdge;
				}
			}
		}

		return contourCombiner.distance();
	}

}
