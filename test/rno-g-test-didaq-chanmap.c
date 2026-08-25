/** Checks the per-station DiDAQ <-> RNO-G channel maps (src/rno-g-didaq-chanmap.c).
 *
 * Needs no hardware: the maps live in the client library precisely so they can be checked
 * (and consulted by readers) without libdidaq.
 */

#include "rno-g.h"
#include <stdlib.h>

#define NCHAN RNO_G_NUM_RADIANT_CHANNELS

static int nfail = 0;

#define CHECK(cond, ...) \
  do { if (!(cond)) { nfail++; printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

/** Both directions must agree and cover every channel exactly once. */
static void check_is_permutation(const rno_g_didaq_chanmap_t * m, int station)
{
  int seen[NCHAN] = {0};

  for (int i = 0; i < NCHAN; i++)
  {
    CHECK(m->to_rno_g[i] < NCHAN, "station %d: to_rno_g[%d] = %d is out of range", station, i, m->to_rno_g[i]);
    if (m->to_rno_g[i] >= NCHAN) return;

    CHECK(!seen[m->to_rno_g[i]], "station %d: RNO-G channel %d is the target of more than one DiDAQ channel",
          station, m->to_rno_g[i]);
    seen[m->to_rno_g[i]] = 1;

    CHECK(m->to_didaq[m->to_rno_g[i]] == i, "station %d: to_didaq[to_rno_g[%d]] = %d, expected %d",
          station, i, m->to_didaq[m->to_rno_g[i]], i);
  }
}

/** Permuting a mask out and back must be the identity. */
static void check_mask_roundtrip(const rno_g_didaq_chanmap_t * m, int station)
{
  uint32_t masks[] = { 0, 0xFFFFFFu, 0x1u, 0x800000u, 0xF000u, 0xF0000u, 0xAAAAAAu, 0x555555u };

  for (unsigned i = 0; i < sizeof(masks) / sizeof(masks[0]); i++)
  {
    uint32_t there = rno_g_didaq_mask_to_rno_g(masks[i], m);
    uint32_t back = rno_g_didaq_mask_to_didaq(there, m);
    CHECK(back == masks[i], "station %d: mask 0x%06x -> 0x%06x -> 0x%06x", station, masks[i], there, back);

    // Permuting must not change how many channels are set.
    CHECK(__builtin_popcount(there) == __builtin_popcount(masks[i]),
          "station %d: mask 0x%06x has %d bits, remapped 0x%06x has %d",
          station, masks[i], __builtin_popcount(masks[i]), there, __builtin_popcount(there));
  }

  // Single bits must land where the table says.
  for (int i = 0; i < NCHAN; i++)
  {
    CHECK(rno_g_didaq_mask_to_rno_g(1u << i, m) == (1u << m->to_rno_g[i]),
          "station %d: DiDAQ channel %d should map to RNO-G channel %d", station, i, m->to_rno_g[i]);
  }
}

/** Several hardware register fields are per-group rather than per-channel, so a permutation is
 *  only expressible at all if it keeps those groups closed: the RNO-G channels of a group must be
 *  exactly the RNO-G images of that same group's DiDAQ inputs. Where that holds, the group is the
 *  same set of numbers on both sides, so an RNO-G-numbered config mask can be converted with
 *  rno_g_didaq_mask_to_didaq() and then sliced up per group without losing bits.
 */
static void check_group_closed(const rno_g_didaq_chanmap_t * m, int station, uint32_t group,
                               const char * what)
{
  uint32_t image = rno_g_didaq_mask_to_rno_g(group, m);
  CHECK(image == group,
        "station %d: the channel map moves channels across the %s boundary "
        "(DiDAQ inputs 0x%06x land on RNO-G channels 0x%06x)", station, what, group, image);
}

/** The two coincidence triggers are each hardwired to one half of the board, and their exclude
 *  masks are 12-bit register fields covering that half. rno-g-acq configures them by permuting a
 *  24-bit RNO-G exclude mask and slicing it into the two fields, which only works if no channel
 *  crosses the halfway line.
 */
static void check_coinc_halves_closed(const rno_g_didaq_chanmap_t * m, int station)
{
  check_group_closed(m, station, 0x000fffu, "coinc[0] / coinc[1] (channels 0-11 vs 12-23)");
  check_group_closed(m, station, 0xfff000u, "coinc[1] / coinc[0] (channels 12-23 vs 0-11)");
}

/** Groups the trigger logic treats as a unit: the four phased-array channels, whose exclude mask
 *  is a 4-bit register field, and the two LPDA quads that DIDAQ_SURF_DOWN/UP_CHANNEL_MASK in
 *  rno-g-didaq.c uses to tell an up-facing COINC1 trigger from a down-facing one.
 */
static void check_trigger_groups_closed(const rno_g_didaq_chanmap_t * m, int station)
{
  check_group_closed(m, station, 0x00000fu, "phased array (channels 0-3)");
  check_group_closed(m, station, 0x00f000u, "SURF DOWN (channels 12-15)");
  check_group_closed(m, station, 0x0f0000u, "SURF UP (channels 16-19)");
}

int main(int argc, char ** argv)
{
  (void) argc; (void) argv;

  // Every station number must yield a usable map, whether or not it has a table entry.
  for (int station = 0; station < 256; station++)
  {
    const rno_g_didaq_chanmap_t * m = rno_g_didaq_chanmap(station);
    CHECK(m != NULL, "station %d: got a NULL map", station);
    if (!m) continue;

    check_is_permutation(m, station);
    check_mask_roundtrip(m, station);
    check_coinc_halves_closed(m, station);
    check_trigger_groups_closed(m, station);
  }

  // A station with no table entry gets the identity map, so old (unmapped) stations and any
  // station added before its wiring is known keep behaving exactly as before.
  const rno_g_didaq_chanmap_t * unknown = rno_g_didaq_chanmap(255);
  for (int i = 0; i < NCHAN; i++)
  {
    CHECK(unknown->to_rno_g[i] == i && unknown->to_didaq[i] == i,
          "unknown station should get the identity map, but channel %d maps to %d", i, unknown->to_rno_g[i]);
  }

  if (nfail) printf("\n%d check(s) failed\n", nfail);
  else printf("All DiDAQ channel map checks passed\n");

  return nfail ? 1 : 0;
}
