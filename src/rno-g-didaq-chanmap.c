/** Per-station DiDAQ <-> RNO-G channel maps.
 *
 * See rno_g_didaq_chanmap() in rno-g.h for what these are for. Only the forward direction
 * (DiDAQ channel -> RNO-G channel) is written out by hand below; the inverse is derived, and
 * validated as a genuine permutation, on first use.
 */

#include "rno-g.h"
#include <pthread.h>

#define NCHAN RNO_G_NUM_RADIANT_CHANNELS

static const uint8_t to_rno_g_identity[NCHAN] =
  {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23};

/* Add one table per station whose wiring is not the identity, e.g.
 *
 *   static const uint8_t to_rno_g_s25[NCHAN] = { ... };
 *
 * with to_rno_g_sNN[didaq_chan] = rno_g_chan, then list it in station_maps below. Stations
 * absent from the table fall back to the identity map.
 */
static const uint8_t to_rno_g_s25[NCHAN] =
  {1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 17, 16, 19, 18, 21, 20, 23, 22};

/** The station number is an int (not uint8_t) so that the fallback entry can use -1, which can
 *  never match a real station. It must stay at index 0.
 */
static const struct
{
  int station;
  const uint8_t * to_rno_g;
} station_maps[] =
{
  { -1, to_rno_g_identity },  //!< fallback for stations with no entry; keep first
  { 25, to_rno_g_s25 },
};

#define NUM_STATION_MAPS ((int) (sizeof(station_maps) / sizeof(station_maps[0])))

static rno_g_didaq_chanmap_t built[NUM_STATION_MAPS];
static pthread_once_t built_once = PTHREAD_ONCE_INIT;

/** Fill in m from a forward table, deriving the inverse. Falls back to the identity (and
 *  complains) if to_rno_g isn't a permutation of 0..NCHAN-1 (i.e., not unique channels).
 */
static void build_chanmap(rno_g_didaq_chanmap_t * m, const uint8_t * to_rno_g, int station)
{
  uint8_t seen[NCHAN] = {0};

  for (int i = 0; i < NCHAN; i++)
  {
    uint8_t out = to_rno_g[i];
    if (out >= NCHAN || seen[out])
    {
      fprintf(stderr, "rno_g_didaq_chanmap: the channel map for station %d is not a permutation "
                      "of 0-%d! Falling back to the identity map.\n", station, NCHAN - 1);
      for (int j = 0; j < NCHAN; j++) m->to_rno_g[j] = m->to_didaq[j] = j;
      return;
    }
    seen[out] = 1;
    m->to_rno_g[i] = out;
    m->to_didaq[out] = i;
  }
}

static void build_all_chanmaps(void)
{
  for (int i = 0; i < NUM_STATION_MAPS; i++)
  {
    build_chanmap(&built[i], station_maps[i].to_rno_g, station_maps[i].station);
  }
}

const rno_g_didaq_chanmap_t * rno_g_didaq_chanmap(uint8_t station)
{
  pthread_once(&built_once, build_all_chanmaps);

  for (int i = 1; i < NUM_STATION_MAPS; i++)
  {
    if (station_maps[i].station == (int) station) return &built[i];
  }

  static int warned = 0;
  if (!warned)
  {
    warned = 1;
    fprintf(stderr, "rno_g_didaq_chanmap: no DiDAQ channel map for station %d, using the "
                    "identity map.\n", station);
  }

  return &built[0];
}

uint32_t rno_g_didaq_mask_to_rno_g(uint32_t didaq_mask, const rno_g_didaq_chanmap_t * m)
{
  uint32_t out = 0;
  for (int i = 0; i < NCHAN; i++)
  {
    if (didaq_mask & (1u << i)) out |= 1u << m->to_rno_g[i];
  }
  return out;
}

uint32_t rno_g_didaq_mask_to_didaq(uint32_t rno_g_mask, const rno_g_didaq_chanmap_t * m)
{
  uint32_t out = 0;
  for (int i = 0; i < NCHAN; i++)
  {
    if (rno_g_mask & (1u << i)) out |= 1u << m->to_didaq[i];
  }
  return out;
}
