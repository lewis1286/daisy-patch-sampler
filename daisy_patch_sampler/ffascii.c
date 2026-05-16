/* ffascii.c — minimal ff_convert / ff_wtoupper for ASCII-only filenames.
 *
 * FatFS requires these functions when _USE_LFN > 0. The full implementation
 * (ccsbcs.c) embeds a large OEM code-page lookup table that exceeds the FLASH
 * budget. Since our filenames are pure ASCII (IR_00.wav … IR_07.wav), the
 * mapping is trivially the identity for code points 0–127.
 */

typedef unsigned int   UINT;
typedef unsigned short WCHAR;

/* OEM ↔ Unicode conversion.
 * dir = 1 : OEM → Unicode
 * dir = 0 : Unicode → OEM
 * ASCII is identical in every OEM code page for code points 0–127.
 * Return 0 to signal "no mapping" for any non-ASCII character. */
WCHAR ff_convert(WCHAR wchar, UINT dir)
{
    (void)dir;
    return (wchar < 0x80u) ? wchar : 0u;
}

/* Unicode character to uppercase.
 * Only ASCII letters need handling for our IR_xx.wav filenames. */
WCHAR ff_wtoupper(WCHAR wchar)
{
    return (wchar >= (WCHAR)'a' && wchar <= (WCHAR)'z')
               ? (WCHAR)(wchar - 32u)
               : wchar;
}
